#pragma once

#include "vi_stream_types.h"

// Stream one X-strip through the sliding window.
// cu_id: 0=forward (Y ascending), 1=reverse (Y descending).
void stream_strip(
    value_t   *value_table,
    const penalty_t *penalty_table,
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int map_x, int map_y,
    int strip_x0,
    int strip_w,
    int cu_id,
    value_t &strip_max_delta);
