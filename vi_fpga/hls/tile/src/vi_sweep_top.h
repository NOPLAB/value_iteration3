#pragma once

#include "vi_types.h"

// Top-level HLS kernel: one sweep of Value Iteration over assigned tiles.
//
// value_table:   DDR, [map_y][map_x][N_THETA], ap_uint<16>. Read/write.
// penalty_table: DDR, [map_y][map_x], ap_uint<16>. Read-only.
// trans_table:   DDR, [N_ACTIONS * N_THETA], packed (dix,diy,dit). Read once.
// map_x, map_y:  map dimensions in cells.
// num_tiles_x/y: number of tiles in each direction.
// cu_id:         0 or 1 — selects checkerboard phase.
// max_delta:     output — maximum value change in this sweep.
extern "C" void vi_sweep(
    value_t *value_table,
    const penalty_t *penalty_table,
    const ap_uint<32> *trans_table,
    int map_x,
    int map_y,
    int num_tiles_x,
    int num_tiles_y,
    int cu_id,
    value_t *max_delta);
