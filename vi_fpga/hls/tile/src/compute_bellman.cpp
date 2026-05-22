#include "compute_bellman.h"

// Saturating add for cost computation.
// If either input is a sentinel (MAX_VALUE / PENALTY_OBSTACLE),
// returns MAX_VALUE. PENALTY_GOAL is treated as 0.
static inline value_t cost_of(value_t nv, penalty_t np_raw) {
    if (nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE) return MAX_VALUE;
    penalty_t np = (np_raw == PENALTY_GOAL) ? (penalty_t)0 : np_raw;
    ap_uint<17> sum = (ap_uint<17>)nv + (ap_uint<17>)np;
    return (sum >= (ap_uint<17>)MAX_VALUE)
           ? (value_t)(MAX_VALUE - 1) : (value_t)sum;
}

void compute_bellman(
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    const penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    const penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    const offset_t delta_table[N_ACTIONS][N_THETA][3],
    int tile_w, int tile_h,
    value_t &max_delta)
{
    // val_buf: complete partition on theta dim (dim=3 in 1-indexed) gives 60
    // separate BRAMs. For the 6 actions we access 3 distinct theta banks
    // (it, it+3, it-3), each serving 2 action reads. Use true dual-port
    // (ram_t2p) so each bank has 2 independent R/W ports; combined with
    // PREFETCH_OLD, bank[it] needs only 2 reads + 1 write which the 2 ports
    // can handle via pipeline scheduling.
    #pragma HLS ARRAY_PARTITION variable=val_buf complete dim=3
    #pragma HLS BIND_STORAGE variable=val_buf type=ram_t2p impl=bram

    // 3 copies of penalty buffer, each serving 2 actions (dual-port).
    #pragma HLS BIND_STORAGE variable=pen_buf_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_1 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_2 type=ram_2p impl=bram

    // Transition table fully in registers.
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0

    // Inter-iteration dependence on val_buf is actually distance >= 3
    // (rotation actions read [it+/-3]), so HLS can pipeline more aggressively.
    #pragma HLS DEPENDENCE variable=val_buf type=inter false

    value_t local_max_delta = 0;

    LOOP_Y: for (int iy = 0; iy < tile_h; iy++) {
        LOOP_X: for (int ix = 0; ix < tile_w; ix++) {
            #pragma HLS LOOP_FLATTEN off
            int by = iy + HALO;
            int bx = ix + HALO;

            // Hoist skip check out of theta loop (pen_buf doesn't depend on it)
            penalty_t cell_pen = pen_buf_0[by][bx];
            bool skip = (cell_pen >= PENALTY_GOAL);

            LOOP_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1

                // The 6 fixed actions use 3 known-distinct theta banks:
                //   actions 0 (forward)  & 1 (backward):   bank[it]
                //   actions 2 (left)     & 4 (fwd-left):   bank[it+3]
                //   actions 3 (right)    & 5 (fwd-right):  bank[it-3]
                // These are provably distinct (offsets 0, +3, -3 mod 60).
                int it_fw = it;
                int it_l  = it + 3;
                if (it_l >= N_THETA) it_l -= N_THETA;
                int it_r  = it - 3;
                if (it_r < 0) it_r += N_THETA;

                value_t old_val = val_buf[by][bx][it];

                // --- Action 0: forward (bank it_fw, port 0) ---
                int ny0 = by + (int)delta_table[0][it][1];
                int nx0 = bx + (int)delta_table[0][it][0];
                value_t  nv0 = val_buf[ny0][nx0][it_fw];
                penalty_t np0 = pen_buf_0[ny0][nx0];
                value_t  c0  = cost_of(nv0, np0);

                // --- Action 1: backward (bank it_fw, port 1) ---
                int ny1 = by + (int)delta_table[1][it][1];
                int nx1 = bx + (int)delta_table[1][it][0];
                value_t  nv1 = val_buf[ny1][nx1][it_fw];
                penalty_t np1 = pen_buf_0[ny1][nx1];
                value_t  c1  = cost_of(nv1, np1);

                // --- Action 2: left, +20deg, no spatial move (bank it_l, port 0) ---
                int ny2 = by;
                int nx2 = bx;
                value_t  nv2 = val_buf[ny2][nx2][it_l];
                penalty_t np2 = pen_buf_1[ny2][nx2];
                value_t  c2  = cost_of(nv2, np2);

                // --- Action 3: right, -20deg, no spatial move (bank it_r, port 0) ---
                int ny3 = by;
                int nx3 = bx;
                value_t  nv3 = val_buf[ny3][nx3][it_r];
                penalty_t np3 = pen_buf_1[ny3][nx3];
                value_t  c3  = cost_of(nv3, np3);

                // --- Action 4: forward-left (bank it_l, port 1) ---
                int ny4 = by + (int)delta_table[4][it][1];
                int nx4 = bx + (int)delta_table[4][it][0];
                value_t  nv4 = val_buf[ny4][nx4][it_l];
                penalty_t np4 = pen_buf_2[ny4][nx4];
                value_t  c4  = cost_of(nv4, np4);

                // --- Action 5: forward-right (bank it_r, port 1) ---
                int ny5 = by + (int)delta_table[5][it][1];
                int nx5 = bx + (int)delta_table[5][it][0];
                value_t  nv5 = val_buf[ny5][nx5][it_r];
                penalty_t np5 = pen_buf_2[ny5][nx5];
                value_t  c5  = cost_of(nv5, np5);

                // Min reduction tree
                value_t min01 = (c0 < c1) ? c0 : c1;
                value_t min23 = (c2 < c3) ? c2 : c3;
                value_t min45 = (c4 < c5) ? c4 : c5;
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
