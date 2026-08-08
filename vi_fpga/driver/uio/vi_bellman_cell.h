#ifndef VI_BELLMAN_CELL_H
#define VI_BELLMAN_CELL_H

/* Shared per-cell Bellman update (16-bit data contract).
   The single C-side definition of the update rule — the host reference
   (vi_reference_c.c) and the mock device (vi_device_mock.c) both call this,
   so the sentinel convention (PENALTY_GOAL 0xFFFE read as 0) cannot drift
   between them. */

#include <stddef.h>
#include <stdint.h>

#include "libvi_sweep.h"

/* Update one (x, y) cell over all theta; returns the cell's max |delta|.
   Obstacle/goal cells (penalty >= 0xFFFE) are skipped and return 0. */
static inline uint16_t vi_bellman_cell(uint16_t *value, const uint16_t *penalty,
                                       const uint32_t *trans,
                                       int map_x, int map_y, int x, int y) {
    uint16_t cell_pen = penalty[y * map_x + x];
    if (cell_pen >= 0xFFFE) return 0;

    uint16_t max_delta = 0;
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
            uint16_t nv = value[nidx];
            uint16_t np_raw = penalty[ny * map_x + nx];
            if (nv == 0xFFFF || np_raw == 0xFFFF) continue;
            uint16_t np = (np_raw == 0xFFFE) ? 0 : np_raw;
            uint32_t s = (uint32_t)nv + np;
            uint16_t c = (s >= 0xFFFF) ? 0xFFFE : (uint16_t)s;
            if (c < best) best = c;
        }
        value[idx] = best;
        uint16_t d = (best > old) ? (best - old) : (old - best);
        if (d > max_delta) max_delta = d;
    }
    return max_delta;
}

#endif /* VI_BELLMAN_CELL_H */
