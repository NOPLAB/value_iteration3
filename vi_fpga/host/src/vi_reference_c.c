#include "vi_reference_c.h"
#include "libvi_sweep.h"

#include <stddef.h>

int vi_reference_run(uint16_t *value, const uint16_t *penalty,
                     const uint32_t *trans,
                     int map_x, int map_y,
                     uint16_t threshold, int max_sweeps) {
    for (int sweep = 0; sweep < max_sweeps; sweep++) {
        uint16_t max_delta = 0;
        for (int y = 0; y < map_y; y++) {
            for (int x = 0; x < map_x; x++) {
                uint16_t cell_pen = penalty[y * map_x + x];
                if (cell_pen >= 0xFFFE) continue;

                for (int it = 0; it < VI_N_THETA; it++) {
                    size_t idx = ((size_t)y * map_x + x) * VI_N_THETA + it;
                    uint16_t old = value[idx];
                    uint16_t best = 0xFFFF;

                    for (int a = 0; a < VI_N_ACTIONS; a++) {
                        uint32_t t = trans[a * VI_N_THETA + it];
                        int8_t dix = (int8_t)(t & 0xFF);
                        int8_t diy = (int8_t)((t >> 8) & 0xFF);
                        int8_t dit = (int8_t)((t >> 16) & 0xFF);
                        int nx = x + dix, ny = y + diy, nt = it + dit;
                        if (nt < 0) nt += VI_N_THETA;
                        if (nt >= VI_N_THETA) nt -= VI_N_THETA;
                        if (nx < 0 || nx >= map_x || ny < 0 || ny >= map_y) continue;

                        size_t nidx = ((size_t)ny * map_x + nx) * VI_N_THETA + nt;
                        uint16_t nvv = value[nidx];
                        uint16_t np_raw = penalty[ny * map_x + nx];
                        if (nvv == 0xFFFF || np_raw == 0xFFFF) continue;
                        uint16_t np = (np_raw == 0xFFFE) ? 0 : np_raw;
                        uint32_t s = (uint32_t)nvv + np;
                        uint16_t c = (s >= 0xFFFF) ? 0xFFFE : (uint16_t)s;
                        if (c < best) best = c;
                    }
                    value[idx] = best;
                    uint16_t d = (best > old) ? (best - old) : (old - best);
                    if (d > max_delta) max_delta = d;
                }
            }
        }
        if (max_delta <= threshold) return sweep + 1;
    }
    return max_sweeps;
}
