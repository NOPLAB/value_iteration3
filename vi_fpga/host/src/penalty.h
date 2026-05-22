#ifndef PENALTY_H
#define PENALTY_H

#include <stdint.h>
#include "map_pgm.h"

/* Build a 16-bit penalty table from a PGM occupancy map.
   - Obstacle cells (pixel >= occupied_thresh*255) → 0xFFFF
   - Within safety_radius of an obstacle → scaled penalty (nearer = higher)
   - Free cells far from obstacles → 0
   - Goal cells (gx, gy) marked 0xFFFE
   pen_out must have map.w*map.h entries. */
void penalty_build(const pgm_map_t *map,
                   int safety_radius,
                   int gx, int gy,
                   uint16_t *pen_out);

#endif
