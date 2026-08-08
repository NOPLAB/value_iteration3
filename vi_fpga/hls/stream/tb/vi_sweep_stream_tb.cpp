#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "../src/vi_sweep_stream_top.h"
#include "vi_reference.h"

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

    vi_ref::build_test_map(pen_ref, val_ref, map_x, map_y, goal_x, goal_y);
    memcpy(pen_hls, pen_ref, map_size * sizeof(uint16_t));
    memcpy(val_hls, val_ref, state_size * sizeof(uint16_t));

    // Transitions (packed, shared with HLS)
    uint32_t trans_packed[vi_ref::N_ACTIONS * vi_ref::N_THETA];
    vi_ref::compute_transitions(xy_res, trans_packed);

    // CPU reference
    int ref_sweeps = vi_ref::run_vi(val_ref, pen_ref, trans_packed,
                                     map_x, map_y, 0, 200);
    printf("  CPU reference: %d sweeps\n", ref_sweeps);

    // HLS kernel — both CUs per sweep (CU0=left half, CU1=right half)
    value_t hls_max_delta;
    int hls_sweeps = 0;
    for (int s = 0; s < 200; s++) {
        value_t d0, d1;
        vi_sweep_stream(
            (value_t *)val_hls,
            (const value_t *)val_hls,
            (const penalty_t *)pen_hls,
            (const ap_uint<32> *)trans_packed,
            map_x, map_y, 0, &d0);
        vi_sweep_stream(
            (value_t *)val_hls,
            (const value_t *)val_hls,
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
