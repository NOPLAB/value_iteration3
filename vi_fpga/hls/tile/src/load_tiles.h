#pragma once

#include "vi_types.h"

// Load a tile (with halo) from DDR into BRAM buffers.
// tile_ox, tile_oy: origin of tile in map coordinates (cells, not including halo).
// Cells outside map boundaries are filled with MAX_VALUE / PENALTY_OBSTACLE.
void load_tile(
    const value_t *value_table,
    const penalty_t *penalty_table,
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    int tile_ox, int tile_oy,
    int map_x, int map_y);
