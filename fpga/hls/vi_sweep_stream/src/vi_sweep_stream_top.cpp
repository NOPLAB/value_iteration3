#include "vi_sweep_stream_top.h"
#include "stream_strip.h"

static void load_transitions(
    const ap_uint<32> *trans_table,
    offset_t delta_table[N_ACTIONS][N_THETA][3])
{
    #pragma HLS INLINE off
    LOAD_TRANS: for (int i = 0; i < TRANS_TABLE_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        ap_uint<32> w = trans_table[i];
        int a = i / N_THETA;
        int t = i % N_THETA;
        delta_table[a][t][0] = (offset_t)(w(7,  0));
        delta_table[a][t][1] = (offset_t)(w(15, 8));
        delta_table[a][t][2] = (offset_t)(w(23, 16));
    }
}

extern "C" void vi_sweep_stream(
    value_t       *value_table,
    const penalty_t *penalty_table,
    const ap_uint<32> *trans_table,
    int map_x,
    int map_y,
    int cu_id,
    value_t *max_delta)
{
    // --- AXI interface pragmas ---
    #pragma HLS INTERFACE m_axi port=value_table   bundle=gmem0 depth=672000000
    #pragma HLS INTERFACE m_axi port=penalty_table  bundle=gmem1 depth=11200000
    #pragma HLS INTERFACE m_axi port=trans_table    bundle=gmem1 depth=360
    #pragma HLS INTERFACE s_axilite port=value_table
    #pragma HLS INTERFACE s_axilite port=penalty_table
    #pragma HLS INTERFACE s_axilite port=trans_table
    #pragma HLS INTERFACE s_axilite port=map_x
    #pragma HLS INTERFACE s_axilite port=map_y
    #pragma HLS INTERFACE s_axilite port=cu_id
    #pragma HLS INTERFACE s_axilite port=max_delta
    #pragma HLS INTERFACE s_axilite port=return

    // 1. Load transition table into registers
    offset_t delta_table[N_ACTIONS][N_THETA][3];
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0
    load_transitions(trans_table, delta_table);

    // 2. Compute strip layout — each CU handles half the strips
    int num_strips = (map_x + STRIP_W_MAX - 1) / STRIP_W_MAX;
    int half_strips = (num_strips + 1) / 2;  // CU0 gets ceil, CU1 gets floor

    value_t global_max_delta = 0;

    // 3. Iterate X-strips (CU0: left half L→R, CU1: right half R→L)
    STRIP_LOOP: for (int si = 0; si < half_strips; si++) {
        #pragma HLS LOOP_TRIPCOUNT min=1 max=35
        int sx;
        if (cu_id == 0)
            sx = si;                       // 0, 1, …, half-1
        else
            sx = num_strips - 1 - si;      // N-1, N-2, …, half
        if (sx < 0 || sx >= num_strips) break;
        int strip_x0 = sx * STRIP_W_MAX;
        int strip_w  = ((strip_x0 + STRIP_W_MAX) > map_x)
                       ? (map_x - strip_x0) : STRIP_W_MAX;

        value_t strip_delta;
        stream_strip(value_table, penalty_table, delta_table,
                     map_x, map_y, strip_x0, strip_w,
                     cu_id, strip_delta);

        if (strip_delta > global_max_delta)
            global_max_delta = strip_delta;
    }

    *max_delta = global_max_delta;
}
