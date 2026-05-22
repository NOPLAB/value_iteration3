#include "load_tiles.h"

void load_transitions(
    const ap_uint<32> *trans_table,
    offset_t delta_table[N_ACTIONS][N_THETA][3])
{
    LOAD_TRANS: for (int i = 0; i < TRANS_TABLE_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        ap_uint<32> w = trans_table[i];
        int a = i / N_THETA;
        int t = i % N_THETA;
        delta_table[a][t][0] = (offset_t)(w(7, 0));    // dix
        delta_table[a][t][1] = (offset_t)(w(15, 8));   // diy
        delta_table[a][t][2] = (offset_t)(w(23, 16));  // dit
    }
}

void load_tile(
    const value_t *value_table,
    const penalty_t *penalty_table,
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    int tile_ox, int tile_oy,
    int map_x, int map_y)
{
    // Strategy: process each row (ly) with 3 phases:
    //   1. Fill the entire row with sentinel values (handles all OOB cells).
    //   2. If the y-row is in bounds, burst-read the in-bounds x range
    //      from DDR for both penalty and value tables (long contiguous bursts).
    //
    // This eliminates the if/else conditional inside the inner loop, which
    // was preventing HLS from inferring large AXI bursts.

    LOAD_V_Y: for (int ly = 0; ly < TILE_H_H; ly++) {
        int gy = tile_oy - HALO + ly;
        bool y_oob = (gy < 0 || gy >= map_y);

        int gx_start = tile_ox - HALO;
        int x0_global = (gx_start < 0) ? 0 : gx_start;
        int x1_global = (gx_start + TILE_W_H > map_x)
                      ? map_x : (gx_start + TILE_W_H);
        int x_count = y_oob ? 0 : (x1_global - x0_global);
        int lx_offset = x0_global - gx_start;
        // Indices [0..lx_offset-1] and [lx_offset+x_count..TILE_W_H-1] are OOB

        // Unconditional fill for the entire row (pipelines cleanly at II=1).
        // The in-bounds cells will be overwritten by DDR loads below.
        FILL_PEN: for (int lx = 0; lx < TILE_W_H; lx++) {
            #pragma HLS PIPELINE II=1
            pen_buf_0[ly][lx] = PENALTY_OBSTACLE;
            pen_buf_1[ly][lx] = PENALTY_OBSTACLE;
            pen_buf_2[ly][lx] = PENALTY_OBSTACLE;
        }
        FILL_VAL_X: for (int lx = 0; lx < TILE_W_H; lx++) {
            FILL_VAL_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1
                val_buf[ly][lx][it] = MAX_VALUE;
            }
        }

        // Burst read in-bounds x range from DDR
        if (x_count > 0) {
            // Burst read penalty range (1 read x_count cycles)
            int pen_base = gy * map_x + x0_global;
            LOAD_PEN: for (int i = 0; i < x_count; i++) {
                #pragma HLS PIPELINE II=1
                penalty_t pen = penalty_table[pen_base + i];
                pen_buf_0[ly][lx_offset + i] = pen;
                pen_buf_1[ly][lx_offset + i] = pen;
                pen_buf_2[ly][lx_offset + i] = pen;
            }

            // Burst read value range (x_count * N_THETA contiguous words)
            int val_base = (gy * map_x + x0_global) * N_THETA;
            LOAD_VAL_X: for (int i = 0; i < x_count; i++) {
                LOAD_VAL_T: for (int it = 0; it < N_THETA; it++) {
                    #pragma HLS PIPELINE II=1
                    val_buf[ly][lx_offset + i][it] =
                        value_table[val_base + i * N_THETA + it];
                }
            }
        }
    }
}
