#include <cstdio>
#include <cstring>
#include "../src/vi_sweep_top.h"
#include "vi_reference.h"

// Test map dimensions (small enough for full BRAM residence)
constexpr int MAP_X = 20;
constexpr int MAP_Y = 20;
constexpr int MAP_SIZE = MAP_X * MAP_Y;
constexpr int STATE_SIZE = MAP_SIZE * vi_ref::N_THETA;
constexpr double XY_RESOLUTION = 0.05;  // meters per cell

int main()
{
    printf("=== Value Iteration HLS C-Simulation Testbench ===\n");
    printf("Map: %d x %d, theta cells: %d, resolution: %.3f m\n",
           MAP_X, MAP_Y, vi_ref::N_THETA, XY_RESOLUTION);

    // Compute packed transition table (shared with HLS)
    uint32_t trans_packed[vi_ref::N_ACTIONS * vi_ref::N_THETA];
    vi_ref::compute_transitions(XY_RESOLUTION, trans_packed);

    // Build test map
    int goal_x = 15, goal_y = 15;
    uint16_t penalty_ref[MAP_SIZE];
    uint16_t value_ref[STATE_SIZE];
    vi_ref::build_test_map(penalty_ref, value_ref, MAP_X, MAP_Y, goal_x, goal_y);

    // Make a copy for HLS
    uint16_t penalty_hls[MAP_SIZE];
    uint16_t value_hls[STATE_SIZE];
    memcpy(penalty_hls, penalty_ref, sizeof(penalty_ref));
    memcpy(value_hls, value_ref, sizeof(value_ref));

    // Run CPU reference
    printf("\nRunning CPU reference...\n");
    int ref_sweeps = vi_ref::run_vi(value_ref, penalty_ref, trans_packed,
                                     MAP_X, MAP_Y, 0, 200);
    printf("  Converged in %d sweeps\n", ref_sweeps);

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
