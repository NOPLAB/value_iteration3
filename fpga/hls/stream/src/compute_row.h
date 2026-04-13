#pragma once

#include "vi_stream_types.h"

// Perform Bellman update for one row in the window.
// val_buf is the full sliding window [WINDOW_ROWS][BUF_W][N_THETA].
// win_center is the circular buffer index of the current row.
// cu_id controls X scan direction: 0=forward (ix++), 1=reverse (ix--).
void compute_row(
    value_t   val_buf[WINDOW_ROWS][BUF_W][N_THETA],
    penalty_t pen_buf_0[WINDOW_ROWS][BUF_W],
    penalty_t pen_buf_1[WINDOW_ROWS][BUF_W],
    penalty_t pen_buf_2[WINDOW_ROWS][BUF_W],
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int win_center,
    int strip_w,
    int cu_id,
    value_t &row_max_delta);
