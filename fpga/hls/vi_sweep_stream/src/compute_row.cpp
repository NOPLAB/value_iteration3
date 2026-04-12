#include "compute_row.h"

static inline value_t cost_of(value_t nv, penalty_t np_raw)
{
    if (nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE) return MAX_VALUE;
    penalty_t np = (np_raw == PENALTY_GOAL) ? (penalty_t)0 : np_raw;
    ap_uint<17> sum = (ap_uint<17>)nv + (ap_uint<17>)np;
    return (sum >= MAX_VALUE) ? (value_t)(MAX_VALUE - 1) : (value_t)sum;
}

void compute_row(
    value_t   val_buf[WINDOW_ROWS][BUF_W][N_THETA],
    penalty_t pen_buf_0[WINDOW_ROWS][BUF_W],
    penalty_t pen_buf_1[WINDOW_ROWS][BUF_W],
    penalty_t pen_buf_2[WINDOW_ROWS][BUF_W],
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int win_center,
    int strip_w,
    int cu_id,
    value_t &row_max_delta)
{
    #pragma HLS INLINE off
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0

    value_t local_max = 0;

    LOOP_X: for (int ix_raw = 0; ix_raw < STRIP_W_MAX; ix_raw++) {
        #pragma HLS LOOP_TRIPCOUNT min=1 max=256
        if (ix_raw >= strip_w) break;

        int ix = (cu_id == 0) ? ix_raw : (strip_w - 1 - ix_raw);
        int bx = ix + HALO_MAX;

        penalty_t cell_pen = pen_buf_0[win_center][bx];
        bool skip = (cell_pen >= PENALTY_GOAL);

        LOOP_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1
            #pragma HLS DEPENDENCE variable=val_buf type=inter false

            value_t old_val = val_buf[win_center][bx][it];

            // --- 6 actions, BRAM-port-scheduled ---
            // Action 0: forward
            int ny0 = (win_center + (int)delta_table[0][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx0 = bx + (int)delta_table[0][it][0];
            int it_fw = it;
            value_t c0 = cost_of(val_buf[ny0][nx0][it_fw], pen_buf_0[ny0][nx0]);

            // Action 1: backward
            int ny1 = (win_center + (int)delta_table[1][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx1 = bx + (int)delta_table[1][it][0];
            value_t c1 = cost_of(val_buf[ny1][nx1][it_fw], pen_buf_0[ny1][nx1]);

            // Action 2: left
            int ny2 = (win_center + (int)delta_table[2][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx2 = bx + (int)delta_table[2][it][0];
            int it_l = it + (int)delta_table[2][it][2];
            it_l = (it_l < 0) ? it_l + N_THETA : (it_l >= N_THETA) ? it_l - N_THETA : it_l;
            value_t c2 = cost_of(val_buf[ny2][nx2][it_l], pen_buf_1[ny2][nx2]);

            // Action 3: right
            int ny3 = (win_center + (int)delta_table[3][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx3 = bx + (int)delta_table[3][it][0];
            int it_r = it + (int)delta_table[3][it][2];
            it_r = (it_r < 0) ? it_r + N_THETA : (it_r >= N_THETA) ? it_r - N_THETA : it_r;
            value_t c3 = cost_of(val_buf[ny3][nx3][it_r], pen_buf_1[ny3][nx3]);

            // Action 4: forward-left
            int ny4 = (win_center + (int)delta_table[4][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx4 = bx + (int)delta_table[4][it][0];
            value_t c4 = cost_of(val_buf[ny4][nx4][it_l], pen_buf_2[ny4][nx4]);

            // Action 5: forward-right
            int ny5 = (win_center + (int)delta_table[5][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx5 = bx + (int)delta_table[5][it][0];
            value_t c5 = cost_of(val_buf[ny5][nx5][it_r], pen_buf_2[ny5][nx5]);

            // Min-reduction tree
            value_t min01 = (c0 < c1) ? c0 : c1;
            value_t min23 = (c2 < c3) ? c2 : c3;
            value_t min45 = (c4 < c5) ? c4 : c5;
            value_t min03 = (min01 < min23) ? min01 : min23;
            value_t min_cost = (min03 < min45) ? min03 : min45;

            // Gauss-Seidel in-place update
            value_t new_val = skip ? old_val : min_cost;
            val_buf[win_center][bx][it] = new_val;

            value_t d = (new_val > old_val) ? (value_t)(new_val - old_val)
                                            : (value_t)(old_val - new_val);
            value_t masked_d = skip ? (value_t)0 : d;
            if (masked_d > local_max) local_max = masked_d;
        }
    }

    row_max_delta = local_max;
}
