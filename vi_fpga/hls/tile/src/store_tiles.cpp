#include "store_tiles.h"

void store_tile(
    value_t *value_table,
    const value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    int tile_ox, int tile_oy,
    int tile_w, int tile_h,
    int map_x)
{
    // Burst write one row at a time. The X*theta dimensions are contiguous
    // in DDR for fixed Y, so we can issue a single tile_w*N_THETA-word burst
    // per row. The PIPELINE pragma is on the flattened inner loop so HLS
    // can infer a long burst and avoid per-(x) restarts.
    STORE_Y: for (int iy = 0; iy < tile_h; iy++) {
        int gy = tile_oy + iy;
        int base_addr = (gy * map_x + tile_ox) * N_THETA;
        int by = iy + HALO;

        STORE_X: for (int ix = 0; ix < tile_w; ix++) {
            int bx = ix + HALO;
            STORE_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1
                value_table[base_addr + ix * N_THETA + it] = val_buf[by][bx][it];
            }
        }
    }
}
