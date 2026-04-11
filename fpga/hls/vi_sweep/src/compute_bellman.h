#pragma once

#include "vi_types.h"

// Bellman update on a tile stored in BRAM.
//
// val_buf: [TILE_H_H][TILE_W_H][N_THETA] — value table tile including halo.
//          Updated in-place (Gauss-Seidel). Only the inner TILE_W x TILE_H
//          region (offset by HALO) is written; halo region is read-only.
// pen_buf: [TILE_H_H][TILE_W_H] — penalty for each cell (theta-independent).
//          PENALTY_OBSTACLE = impassable, PENALTY_GOAL = goal (skip update).
// delta_table: [N_ACTIONS][N_THETA][3] — (dix, diy, dit) offsets.
// tile_w, tile_h: actual tile dimensions (may be < TILE_W at map edge).
// max_delta: output — maximum |V_new - V_old| across all states in this tile.
void compute_bellman(
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    const penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    const offset_t delta_table[N_ACTIONS][N_THETA][3],
    int tile_w, int tile_h,
    value_t &max_delta);
