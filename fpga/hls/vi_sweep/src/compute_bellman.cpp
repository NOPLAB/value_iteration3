#include "compute_bellman.h"

void compute_bellman(
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    const penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    const offset_t delta_table[N_ACTIONS][N_THETA][3],
    int tile_w, int tile_h,
    value_t &max_delta)
{
    // val_buf: complete partition on theta dim for 6 parallel reads.
    // With N_THETA=60 banks, each dual-port, 6 reads are served by 3 bank pairs.
    // (forward/backward share theta bank, left/fwd-left share, right/fwd-right share)
    #pragma HLS ARRAY_PARTITION variable=val_buf complete dim=3
    #pragma HLS BIND_STORAGE variable=val_buf type=ram_2p impl=bram

    // 3 copies of penalty buffer for 6 parallel reads (2 reads per copy)
    #pragma HLS BIND_STORAGE variable=pen_buf_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_1 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_2 type=ram_2p impl=bram

    // Transition table fully in registers
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0

    value_t local_max_delta = 0;

    LOOP_Y: for (int iy = 0; iy < tile_h; iy++) {
        LOOP_X: for (int ix = 0; ix < tile_w; ix++) {
            // Penalty for this cell (used to check skip)
            int by = iy + HALO;
            int bx = ix + HALO;
            penalty_t cell_pen = pen_buf_0[by][bx];
            bool skip = (cell_pen >= PENALTY_GOAL);

            LOOP_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1

                value_t old_val = val_buf[by][bx][it];

                // Compute neighbor coordinates for all 6 actions
                // Actions 0,1 -> pen_buf_0; 2,3 -> pen_buf_1; 4,5 -> pen_buf_2
                value_t costs[N_ACTIONS];
                #pragma HLS ARRAY_PARTITION variable=costs complete

                for (int a = 0; a < N_ACTIONS; a++) {
                    #pragma HLS UNROLL

                    int ny = by + (int)delta_table[a][it][1];
                    int nx = bx + (int)delta_table[a][it][0];
                    int nt_raw = it + (int)delta_table[a][it][2];
                    int nt = (nt_raw < 0) ? (nt_raw + N_THETA)
                           : (nt_raw >= N_THETA) ? (nt_raw - N_THETA)
                           : nt_raw;

                    value_t nv = val_buf[ny][nx][nt];

                    penalty_t np_raw;
                    if (a < 2)      np_raw = pen_buf_0[ny][nx];
                    else if (a < 4) np_raw = pen_buf_1[ny][nx];
                    else            np_raw = pen_buf_2[ny][nx];

                    if (nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE) {
                        costs[a] = MAX_VALUE;
                    } else {
                        // PENALTY_GOAL marks the goal cell — penalty to enter is 0
                        penalty_t np = (np_raw == PENALTY_GOAL) ? (penalty_t)0 : np_raw;
                        ap_uint<17> sum = (ap_uint<17>)nv + (ap_uint<17>)np;
                        costs[a] = (sum >= (ap_uint<17>)MAX_VALUE)
                                 ? (value_t)(MAX_VALUE - 1) : (value_t)sum;
                    }
                }

                // Find minimum cost across 6 actions (reduction tree)
                value_t min01 = (costs[0] < costs[1]) ? costs[0] : costs[1];
                value_t min23 = (costs[2] < costs[3]) ? costs[2] : costs[3];
                value_t min45 = (costs[4] < costs[5]) ? costs[4] : costs[5];
                value_t min03 = (min01 < min23) ? min01 : min23;
                value_t min_cost = (min03 < min45) ? min03 : min45;

                // Conditional update (skip obstacles and goals)
                value_t new_val = skip ? old_val : min_cost;
                val_buf[by][bx][it] = new_val;

                // Delta tracking
                value_t d = (new_val > old_val) ? (value_t)(new_val - old_val)
                                                : (value_t)(old_val - new_val);
                value_t masked_d = skip ? (value_t)0 : d;
                if (masked_d > local_max_delta) {
                    local_max_delta = masked_d;
                }
            }
        }
    }

    max_delta = local_max_delta;
}
