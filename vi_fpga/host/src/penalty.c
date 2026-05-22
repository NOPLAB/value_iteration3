#include "penalty.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void penalty_build(const pgm_map_t *map, int safety_radius,
                   int gx, int gy, uint16_t *pen_out) {
    int w = map->w, h = map->h;
    uint8_t occ_threshold =
        (uint8_t)(map->occupied_thresh > 0 ? map->occupied_thresh * 255 : 205);

    /* Initialise: obstacles = 0xFFFF, rest = 0. */
    for (int i = 0; i < w * h; i++) {
        pen_out[i] = (map->pixels[i] >= occ_threshold) ? 0xFFFF : 0;
    }

    /* Inflate obstacles within safety_radius with scaled penalty. */
    int r = safety_radius;
    if (r <= 0) goto goal;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (pen_out[y * w + x] != 0xFFFF) continue;
            /* Stamp a circular kernel of penalties around (x, y). */
            for (int dy = -r; dy <= r; dy++) {
                int ny = y + dy; if (ny < 0 || ny >= h) continue;
                for (int dx = -r; dx <= r; dx++) {
                    int nx = x + dx; if (nx < 0 || nx >= w) continue;
                    int d2 = dx*dx + dy*dy;
                    if (d2 > r*r) continue;
                    if (pen_out[ny * w + nx] == 0xFFFF) continue;
                    double ratio = 1.0 - sqrt((double)d2) / (double)r;
                    uint16_t p = (uint16_t)(ratio * 1000.0);
                    if (p > pen_out[ny * w + nx]) pen_out[ny * w + nx] = p;
                }
            }
        }
    }

goal:
    if (gx >= 0 && gx < w && gy >= 0 && gy < h) {
        pen_out[gy * w + gx] = 0xFFFE;
    }
}
