#pragma once

#include "vi_types.h"

// Store the inner tile region (excluding halo) back to DDR.
void store_tile(
    value_t *value_table,
    const value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    int tile_ox, int tile_oy,
    int tile_w, int tile_h,
    int map_x);
