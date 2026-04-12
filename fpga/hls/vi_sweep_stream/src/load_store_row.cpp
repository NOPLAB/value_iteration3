#include "load_store_row.h"

void load_row(
    value_t   val_row[BUF_W][N_THETA],
    penalty_t pen_row_0[BUF_W],
    penalty_t pen_row_1[BUF_W],
    penalty_t pen_row_2[BUF_W],
    const value_t   *value_table,
    const penalty_t *penalty_table,
    int gy,
    int strip_x0,
    int strip_w,
    int map_x, int map_y)
{
    int buf_w = strip_w + 2 * HALO_MAX;
    int gx_start = strip_x0 - HALO_MAX;
    bool y_oob = (gy < 0 || gy >= map_y);

    // Phase A: Fill entire row with sentinels
    FILL_PEN: for (int lx = 0; lx < BUF_W; lx++) {
        #pragma HLS PIPELINE II=1
        penalty_t p = PENALTY_OBSTACLE;
        pen_row_0[lx] = p;
        pen_row_1[lx] = p;
        pen_row_2[lx] = p;
    }
    FILL_VAL_X: for (int lx = 0; lx < BUF_W; lx++) {
        FILL_VAL_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1
            val_row[lx][it] = MAX_VALUE;
        }
    }

    if (y_oob) return;

    // Phase B: Compute in-bounds X range
    int x0_global = (gx_start < 0) ? 0 : gx_start;
    int x1_global = (gx_start + buf_w > map_x) ? map_x : (gx_start + buf_w);
    int x_count = x1_global - x0_global;
    int lx_offset = x0_global - gx_start;

    if (x_count <= 0) return;

    // Phase C: Burst-read penalty
    int pen_base = gy * map_x + x0_global;
    LOAD_PEN: for (int i = 0; i < x_count; i++) {
        #pragma HLS PIPELINE II=1
        penalty_t p = penalty_table[pen_base + i];
        pen_row_0[lx_offset + i] = p;
        pen_row_1[lx_offset + i] = p;
        pen_row_2[lx_offset + i] = p;
    }

    // Phase D: Burst-read value (contiguous: x * N_THETA)
    int val_base = (gy * map_x + x0_global) * N_THETA;
    LOAD_VAL_X: for (int i = 0; i < x_count; i++) {
        LOAD_VAL_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1
            val_row[lx_offset + i][it] = value_table[val_base + i * N_THETA + it];
        }
    }
}

void store_row(
    const value_t val_row[BUF_W][N_THETA],
    value_t *value_table,
    int gy,
    int strip_x0,
    int strip_w,
    int map_x)
{
    int val_base = (gy * map_x + strip_x0) * N_THETA;

    STORE_X: for (int ix = 0; ix < strip_w; ix++) {
        int bx = ix + HALO_MAX;
        STORE_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1
            value_table[val_base + ix * N_THETA + it] = val_row[bx][it];
        }
    }
}
