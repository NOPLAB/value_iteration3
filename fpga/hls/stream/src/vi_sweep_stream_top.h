#pragma once

#include "vi_stream_types.h"

extern "C" void vi_sweep_stream(
    value_t       *value_table,
    const value_t *value_table_rd,
    const penalty_t *penalty_table,
    const ap_uint<32> *trans_table,
    int map_x,
    int map_y,
    int cu_id,
    value_t *max_delta);
