#pragma once

#include "vi_stream_types.h"

// Load one row (with halo) from DDR into the given window slot.
// If gy is out of bounds, fills with MAX_VALUE / PENALTY_OBSTACLE.
void load_row(
    value_t   val_row[BUF_W][N_THETA],
    penalty_t pen_row_0[BUF_W],
    penalty_t pen_row_1[BUF_W],
    penalty_t pen_row_2[BUF_W],
    const value_t   *value_table,
    const penalty_t *penalty_table,
    int gy,
    int strip_x0,
    int strip_w,
    int map_x, int map_y);

// Store one row (inner cells only, no halo) back to DDR.
void store_row(
    const value_t val_row[BUF_W][N_THETA],
    value_t *value_table,
    int gy,
    int strip_x0,
    int strip_w,
    int map_x);
