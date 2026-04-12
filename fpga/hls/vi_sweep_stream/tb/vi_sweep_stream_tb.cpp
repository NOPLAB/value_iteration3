#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "../src/vi_sweep_stream_top.h"
#include "vi_reference.h"

static void build_test_map(
    uint16_t *penalty, uint16_t *value,
    int map_x, int map_y, int goal_x, int goal_y)
{
    int map_size = map_x * map_y;
    int state_size = map_size * vi_ref::N_THETA;

    for (int i = 0; i < map_size; i++) penalty[i] = 0;

    // Border obstacles
    for (int x = 0; x < map_x; x++) {
        penalty[0 * map_x + x] = vi_ref::PENALTY_OBSTACLE;
        penalty[(map_y - 1) * map_x + x] = vi_ref::PENALTY_OBSTACLE;
    }
    for (int y = 0; y < map_y; y++) {
        penalty[y * map_x + 0] = vi_ref::PENALTY_OBSTACLE;
        penalty[y * map_x + (map_x - 1)] = vi_ref::PENALTY_OBSTACLE;
    }

    // L-shaped obstacle
    for (int x = 5; x <= std::min(12, map_x - 2); x++)
        penalty[10 * map_x + x] = vi_ref::PENALTY_OBSTACLE;
    for (int y = 6; y <= 10; y++)
        if (12 < map_x)
            penalty[y * map_x + 12] = vi_ref::PENALTY_OBSTACLE;

    // Safety penalty near obstacles
    for (int y = 1; y < map_y - 1; y++)
        for (int x = 1; x < map_x - 1; x++) {
            if (penalty[y * map_x + x] == vi_ref::PENALTY_OBSTACLE) continue;
            bool near = false;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    if (penalty[(y+dy)*map_x + (x+dx)] == vi_ref::PENALTY_OBSTACLE)
                        near = true;
            if (near) penalty[y * map_x + x] = 100;
        }

    // Goal
    penalty[goal_y * map_x + goal_x] = vi_ref::PENALTY_GOAL;

    // Value init
    for (int i = 0; i < state_size; i++) value[i] = vi_ref::MAX_VALUE;
    for (int it = 0; it < vi_ref::N_THETA; it++)
        value[(goal_y * map_x + goal_x) * vi_ref::N_THETA + it] = 0;
}

static void pack_transitions(
    const vi_ref::TransitionEntry trans[vi_ref::N_ACTIONS][vi_ref::N_THETA],
    uint32_t *packed)
{
    for (int a = 0; a < vi_ref::N_ACTIONS; a++)
        for (int it = 0; it < vi_ref::N_THETA; it++) {
            uint32_t w = ((uint32_t)(uint8_t)trans[a][it].dix)
                       | ((uint32_t)(uint8_t)trans[a][it].diy << 8)
                       | ((uint32_t)(uint8_t)trans[a][it].dit << 16);
            packed[a * vi_ref::N_THETA + it] = w;
        }
}

static int run_test(const char *name, int map_x, int map_y,
                    int goal_x, int goal_y, double xy_res)
{
    printf("\n=== Test: %s (%dx%d, goal=(%d,%d), res=%.3f) ===\n",
           name, map_x, map_y, goal_x, goal_y, xy_res);

    int map_size   = map_x * map_y;
    int state_size = map_size * vi_ref::N_THETA;

    uint16_t *pen_ref = new uint16_t[map_size];
    uint16_t *val_ref = new uint16_t[state_size];
    uint16_t *pen_hls = new uint16_t[map_size];
    uint16_t *val_hls = new uint16_t[state_size];

    build_test_map(pen_ref, val_ref, map_x, map_y, goal_x, goal_y);
    memcpy(pen_hls, pen_ref, map_size * sizeof(uint16_t));
    memcpy(val_hls, val_ref, state_size * sizeof(uint16_t));

    // Transitions
    vi_ref::TransitionEntry trans[vi_ref::N_ACTIONS][vi_ref::N_THETA];
    vi_ref::compute_transitions(xy_res, trans);
    uint32_t trans_packed[vi_ref::N_ACTIONS * vi_ref::N_THETA];
    pack_transitions(trans, trans_packed);

    // CPU reference
    int ref_sweeps = vi_ref::run_vi(val_ref, pen_ref, trans,
                                     map_x, map_y, 0, 200);
    printf("  CPU reference: %d sweeps\n", ref_sweeps);

    // HLS kernel — both CUs per sweep (CU0=left half, CU1=right half)
    value_t hls_max_delta;
    int hls_sweeps = 0;
    for (int s = 0; s < 200; s++) {
        value_t d0, d1;
        vi_sweep_stream(
            (value_t *)val_hls,
            (const penalty_t *)pen_hls,
            (const ap_uint<32> *)trans_packed,
            map_x, map_y, 0, &d0);
        vi_sweep_stream(
            (value_t *)val_hls,
            (const penalty_t *)pen_hls,
            (const ap_uint<32> *)trans_packed,
            map_x, map_y, 1, &d1);
        hls_max_delta = (d0 > d1) ? d0 : d1;
        hls_sweeps++;
        if ((uint16_t)hls_max_delta == 0) break;
    }
    printf("  HLS kernel: %d sweeps, final_delta=%d\n",
           hls_sweeps, (int)(uint16_t)hls_max_delta);

    // Verify
    int mismatch = 0;
    int checked = 0;
    for (int iy = 0; iy < map_y; iy++)
        for (int ix = 0; ix < map_x; ix++) {
            if (pen_ref[iy * map_x + ix] >= vi_ref::PENALTY_GOAL) continue;
            for (int it = 0; it < vi_ref::N_THETA; it++) {
                int idx = (iy * map_x + ix) * vi_ref::N_THETA + it;
                checked++;
                int diff = abs((int)val_ref[idx] - (int)val_hls[idx]);
                if (diff > 1) {
                    if (mismatch < 5)
                        printf("  MISMATCH (%d,%d,t=%d): ref=%u hls=%u\n",
                               ix, iy, it, val_ref[idx], val_hls[idx]);
                    mismatch++;
                }
            }
        }

    // Propagation check
    int finite = 0, total_free = 0;
    for (int iy = 0; iy < map_y; iy++)
        for (int ix = 0; ix < map_x; ix++) {
            if (pen_hls[iy * map_x + ix] >= vi_ref::PENALTY_GOAL) continue;
            total_free++;
            for (int it = 0; it < vi_ref::N_THETA; it++)
                if (val_hls[(iy * map_x + ix) * vi_ref::N_THETA + it] < vi_ref::MAX_VALUE) {
                    finite++;
                    break;
                }
        }

    printf("  Checked %d states, %d mismatches\n", checked, mismatch);
    printf("  Propagation: %d / %d free cells\n", finite, total_free);

    if (finite < total_free / 2) {
        printf("  FAIL: propagation insufficient\n");
        mismatch++;
    }

    delete[] pen_ref; delete[] val_ref;
    delete[] pen_hls; delete[] val_hls;

    return mismatch;
}

int main()
{
    printf("=== vi_sweep_stream C-Simulation Testbench ===\n");

    int errors = 0;

    // Test A: 20x20, fits in 1 strip (20 < 256)
    errors += run_test("small_single_strip", 20, 20, 15, 15, 0.05);

    // Test B: 300x20, forces 2 strips (300 > 256)
    errors += run_test("wide_multi_strip", 300, 20, 280, 15, 0.05);

    if (errors > 0) {
        printf("\n*** TESTBENCH FAILED (%d errors) ***\n", errors);
        return 1;
    }
    printf("\n*** TESTBENCH PASSED ***\n");
    return 0;
}
