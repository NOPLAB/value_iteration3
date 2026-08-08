#include "vi_reference.h"

// Share the exact host/driver implementations instead of copying them.
// Requires -I driver/uio and -I host/src (see tcl/hls_common.tcl).
#include "transitions.c"      // transitions_compute()
#include "vi_reference_c.c"   // vi_reference_run() over the shared vi_bellman_cell.h

namespace vi_ref {

void compute_transitions(double xy_resolution, uint32_t *out) {
    transitions_compute(xy_resolution, out);
}

int run_vi(uint16_t *value_table, const uint16_t *penalty_table,
           const uint32_t *trans, int map_x, int map_y,
           uint16_t threshold, int max_sweeps)
{
    return vi_reference_run(value_table, penalty_table, trans,
                            map_x, map_y, threshold, max_sweeps);
}

void build_test_map(uint16_t *penalty, uint16_t *value,
                    int map_x, int map_y, int goal_x, int goal_y)
{
    int map_size = map_x * map_y;
    int state_size = map_size * N_THETA;

    for (int i = 0; i < map_size; i++) penalty[i] = 0;

    // Border obstacles
    for (int x = 0; x < map_x; x++) {
        penalty[0 * map_x + x] = PENALTY_OBSTACLE;
        penalty[(map_y - 1) * map_x + x] = PENALTY_OBSTACLE;
    }
    for (int y = 0; y < map_y; y++) {
        penalty[y * map_x + 0] = PENALTY_OBSTACLE;
        penalty[y * map_x + (map_x - 1)] = PENALTY_OBSTACLE;
    }

    // L-shaped obstacle
    int x_end = (map_x - 2 < 12) ? map_x - 2 : 12;
    for (int x = 5; x <= x_end; x++)
        penalty[10 * map_x + x] = PENALTY_OBSTACLE;
    for (int y = 6; y <= 10; y++)
        if (12 < map_x)
            penalty[y * map_x + 12] = PENALTY_OBSTACLE;

    // Safety penalty near obstacles
    for (int y = 1; y < map_y - 1; y++)
        for (int x = 1; x < map_x - 1; x++) {
            if (penalty[y * map_x + x] == PENALTY_OBSTACLE) continue;
            bool near = false;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    if (penalty[(y + dy) * map_x + (x + dx)] == PENALTY_OBSTACLE)
                        near = true;
            if (near) penalty[y * map_x + x] = 100;
        }

    // Goal
    penalty[goal_y * map_x + goal_x] = PENALTY_GOAL;

    // Value init
    for (int i = 0; i < state_size; i++) value[i] = MAX_VALUE;
    for (int it = 0; it < N_THETA; it++)
        value[(goal_y * map_x + goal_x) * N_THETA + it] = 0;
}

} // namespace vi_ref
