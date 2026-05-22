#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "../src/vi_sweep_top.h"
#include "vi_reference.h"

// Test map dimensions (small enough for full BRAM residence)
constexpr int MAP_X = 20;
constexpr int MAP_Y = 20;
constexpr int MAP_SIZE = MAP_X * MAP_Y;
constexpr int STATE_SIZE = MAP_SIZE * vi_ref::N_THETA;
constexpr double XY_RESOLUTION = 0.05;  // meters per cell

// Build a simple test map with obstacles and a goal
static void build_test_map(
    uint16_t *penalty_table,
    uint16_t *value_table,
    int goal_x, int goal_y)
{
    // Initialize all cells as free (penalty = 0)
    for (int i = 0; i < MAP_SIZE; i++)
        penalty_table[i] = 0;

    // Add border obstacles
    for (int x = 0; x < MAP_X; x++) {
        penalty_table[0 * MAP_X + x] = vi_ref::PENALTY_OBSTACLE;
        penalty_table[(MAP_Y - 1) * MAP_X + x] = vi_ref::PENALTY_OBSTACLE;
    }
    for (int y = 0; y < MAP_Y; y++) {
        penalty_table[y * MAP_X + 0] = vi_ref::PENALTY_OBSTACLE;
        penalty_table[y * MAP_X + (MAP_X - 1)] = vi_ref::PENALTY_OBSTACLE;
    }

    // Add an L-shaped obstacle in the middle
    for (int x = 5; x <= 12; x++)
        penalty_table[10 * MAP_X + x] = vi_ref::PENALTY_OBSTACLE;
    for (int y = 6; y <= 10; y++)
        penalty_table[y * MAP_X + 12] = vi_ref::PENALTY_OBSTACLE;

    // Add safety penalty near obstacles (penalty = 100)
    for (int y = 1; y < MAP_Y - 1; y++) {
        for (int x = 1; x < MAP_X - 1; x++) {
            if (penalty_table[y * MAP_X + x] == vi_ref::PENALTY_OBSTACLE) continue;
            // Check 4-neighbors for obstacle adjacency
            bool near_obs = false;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    if (penalty_table[(y+dy) * MAP_X + (x+dx)] == vi_ref::PENALTY_OBSTACLE)
                        near_obs = true;
            if (near_obs)
                penalty_table[y * MAP_X + x] = 100;
        }
    }

    // Set goal cells
    penalty_table[goal_y * MAP_X + goal_x] = vi_ref::PENALTY_GOAL;

    // Initialize value table: goal = 0, others = MAX_VALUE
    for (int i = 0; i < STATE_SIZE; i++)
        value_table[i] = vi_ref::MAX_VALUE;

    for (int it = 0; it < vi_ref::N_THETA; it++) {
        int idx = (goal_y * MAP_X + goal_x) * vi_ref::N_THETA + it;
        value_table[idx] = 0;
    }
}

// Pack transition table for HLS (3 int8 -> 1 uint32)
static void pack_transitions(
    const vi_ref::TransitionEntry trans[vi_ref::N_ACTIONS][vi_ref::N_THETA],
    uint32_t *packed)
{
    for (int a = 0; a < vi_ref::N_ACTIONS; a++) {
        for (int it = 0; it < vi_ref::N_THETA; it++) {
            uint32_t w = 0;
            w |= ((uint32_t)(uint8_t)trans[a][it].dix) << 0;
            w |= ((uint32_t)(uint8_t)trans[a][it].diy) << 8;
            w |= ((uint32_t)(uint8_t)trans[a][it].dit) << 16;
            packed[a * vi_ref::N_THETA + it] = w;
        }
    }
}

int main()
{
    printf("=== Value Iteration HLS C-Simulation Testbench ===\n");
    printf("Map: %d x %d, theta cells: %d, resolution: %.3f m\n",
           MAP_X, MAP_Y, vi_ref::N_THETA, XY_RESOLUTION);

    // Compute transition table
    vi_ref::TransitionEntry trans[vi_ref::N_ACTIONS][vi_ref::N_THETA];
    vi_ref::compute_transitions(XY_RESOLUTION, trans);

    printf("\nTransition table (sample, action=0 forward):\n");
    for (int it = 0; it < 5; it++)
        printf("  theta=%d: dix=%d diy=%d dit=%d\n",
               it, trans[0][it].dix, trans[0][it].diy, trans[0][it].dit);

    // Build test map
    int goal_x = 15, goal_y = 15;
    uint16_t penalty_ref[MAP_SIZE];
    uint16_t value_ref[STATE_SIZE];
    build_test_map(penalty_ref, value_ref, goal_x, goal_y);

    // Make a copy for HLS
    uint16_t penalty_hls[MAP_SIZE];
    uint16_t value_hls[STATE_SIZE];
    memcpy(penalty_hls, penalty_ref, sizeof(penalty_ref));
    memcpy(value_hls, value_ref, sizeof(value_ref));

    // Run CPU reference
    printf("\nRunning CPU reference...\n");
    int ref_sweeps = vi_ref::run_vi(value_ref, penalty_ref, trans,
                                     MAP_X, MAP_Y, 0, 200);
    printf("  Converged in %d sweeps\n", ref_sweeps);

    // Pack transition table for HLS
    uint32_t trans_packed[vi_ref::N_ACTIONS * vi_ref::N_THETA];
    pack_transitions(trans, trans_packed);

    // Run HLS kernel (multiple sweeps to converge)
    printf("\nRunning HLS kernel...\n");
    int num_tiles_x = (MAP_X + TILE_W - 1) / TILE_W;
    int num_tiles_y = (MAP_Y + TILE_H - 1) / TILE_H;
    printf("  Tiles: %d x %d\n", num_tiles_x, num_tiles_y);

    value_t hls_max_delta;
    int hls_sweeps = 0;
    for (int s = 0; s < 200; s++) {
        // Single CU (cu_id=0), process ALL tiles (no checkerboard for small map)
        vi_sweep(
            (value_t *)value_hls,
            (const penalty_t *)penalty_hls,
            (const ap_uint<32> *)trans_packed,
            MAP_X, MAP_Y,
            num_tiles_x, num_tiles_y,
            0,  // cu_id
            &hls_max_delta);

        hls_sweeps++;
        if ((uint16_t)hls_max_delta == 0) break;
    }
    printf("  Converged in %d sweeps, final max_delta=%d\n",
           hls_sweeps, (int)(uint16_t)hls_max_delta);

    // Compare results
    printf("\n=== Verification ===\n");
    int mismatch_count = 0;
    int checked = 0;
    for (int iy = 0; iy < MAP_Y; iy++) {
        for (int ix = 0; ix < MAP_X; ix++) {
            if (penalty_ref[iy * MAP_X + ix] >= vi_ref::PENALTY_GOAL) continue;
            for (int it = 0; it < vi_ref::N_THETA; it++) {
                int idx = (iy * MAP_X + ix) * vi_ref::N_THETA + it;
                uint16_t ref_v = value_ref[idx];
                uint16_t hls_v = value_hls[idx];
                checked++;

                // Allow small tolerance (tile boundary Gauss-Seidel ordering differs)
                int diff = (int)ref_v - (int)hls_v;
                if (diff < 0) diff = -diff;
                if (diff > 1) {
                    if (mismatch_count < 10) {
                        printf("  MISMATCH at (%d,%d,t=%d): ref=%u hls=%u diff=%d\n",
                               ix, iy, it, ref_v, hls_v, diff);
                    }
                    mismatch_count++;
                }
            }
        }
    }

    printf("\nChecked %d states, %d mismatches\n", checked, mismatch_count);

    // Verify goal state unchanged
    for (int it = 0; it < vi_ref::N_THETA; it++) {
        int idx = (goal_y * MAP_X + goal_x) * vi_ref::N_THETA + it;
        if (value_hls[idx] != 0) {
            printf("  FAIL: goal state (%d,%d,t=%d) value=%d (expected 0)\n",
                   goal_x, goal_y, it, (int)value_hls[idx]);
            mismatch_count++;
        }
    }

    // Verify obstacle states unchanged
    for (int iy = 0; iy < MAP_Y; iy++) {
        for (int ix = 0; ix < MAP_X; ix++) {
            if (penalty_hls[iy * MAP_X + ix] != vi_ref::PENALTY_OBSTACLE) continue;
            for (int it = 0; it < vi_ref::N_THETA; it++) {
                int idx = (iy * MAP_X + ix) * vi_ref::N_THETA + it;
                if (value_hls[idx] != vi_ref::MAX_VALUE) {
                    printf("  FAIL: obstacle (%d,%d,t=%d) value=%d (expected MAX)\n",
                           ix, iy, it, (int)value_hls[idx]);
                    mismatch_count++;
                }
            }
        }
    }

    // Verify value propagation: count cells with finite (non-MAX) values
    int finite_count = 0;
    int total_free = 0;
    for (int iy = 0; iy < MAP_Y; iy++) {
        for (int ix = 0; ix < MAP_X; ix++) {
            uint16_t p = penalty_hls[iy * MAP_X + ix];
            if (p >= vi_ref::PENALTY_GOAL) continue;  // skip goals and obstacles
            total_free++;
            // Count if any theta for this cell has a finite value
            bool has_finite = false;
            for (int it = 0; it < vi_ref::N_THETA; it++) {
                int idx = (iy * MAP_X + ix) * vi_ref::N_THETA + it;
                if (value_hls[idx] < vi_ref::MAX_VALUE) {
                    has_finite = true;
                    break;
                }
            }
            if (has_finite) finite_count++;
        }
    }
    printf("Propagation: %d / %d free cells reached (finite value)\n",
           finite_count, total_free);
    if (finite_count < total_free / 2) {
        printf("  FAIL: value propagation insufficient (less than 50%% of free cells)\n");
        mismatch_count++;
    }

    if (mismatch_count > 0) {
        printf("\nTESTBENCH FAILED (%d errors)\n", mismatch_count);
        return 1;
    }

    printf("\nTESTBENCH PASSED\n");
    return 0;
}
