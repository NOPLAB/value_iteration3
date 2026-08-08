#include "vi_reference_c.h"
#include "libvi_sweep.h"
#include "vi_bellman_cell.h"

int vi_reference_run(uint16_t *value, const uint16_t *penalty,
                     const uint32_t *trans,
                     int map_x, int map_y,
                     uint16_t threshold, int max_sweeps) {
    for (int sweep = 0; sweep < max_sweeps; sweep++) {
        uint16_t max_delta = 0;
        for (int y = 0; y < map_y; y++) {
            for (int x = 0; x < map_x; x++) {
                uint16_t d = vi_bellman_cell(value, penalty, trans,
                                             map_x, map_y, x, y);
                if (d > max_delta) max_delta = d;
            }
        }
        if (max_delta <= threshold) return sweep + 1;
    }
    return max_sweeps;
}
