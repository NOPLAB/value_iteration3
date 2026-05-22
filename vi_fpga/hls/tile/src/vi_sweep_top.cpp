#include "vi_sweep_top.h"
#include "load_tiles.h"
#include "compute_bellman.h"
#include "store_tiles.h"

extern "C" void vi_sweep(
    value_t *value_table,
    const penalty_t *penalty_table,
    const ap_uint<32> *trans_table,
    int map_x,
    int map_y,
    int num_tiles_x,
    int num_tiles_y,
    int cu_id,
    value_t *max_delta)
{
    // -----------------------------------------------------------------------
    // AXI interface pragmas
    // -----------------------------------------------------------------------
    #pragma HLS INTERFACE m_axi port=value_table   bundle=gmem0 offset=slave depth=672000000
    #pragma HLS INTERFACE m_axi port=penalty_table bundle=gmem1 offset=slave depth=11200000
    #pragma HLS INTERFACE m_axi port=trans_table   bundle=gmem1 offset=slave depth=360

    #pragma HLS INTERFACE s_axilite port=map_x
    #pragma HLS INTERFACE s_axilite port=map_y
    #pragma HLS INTERFACE s_axilite port=num_tiles_x
    #pragma HLS INTERFACE s_axilite port=num_tiles_y
    #pragma HLS INTERFACE s_axilite port=cu_id
    #pragma HLS INTERFACE s_axilite port=max_delta
    #pragma HLS INTERFACE s_axilite port=return

    // -----------------------------------------------------------------------
    // Load transition table (once per invocation)
    // -----------------------------------------------------------------------
    offset_t delta_table[N_ACTIONS][N_THETA][3];
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0

    load_transitions(trans_table, delta_table);

    // -----------------------------------------------------------------------
    // BRAM tile buffers
    // -----------------------------------------------------------------------
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA];
    #pragma HLS ARRAY_PARTITION variable=val_buf complete dim=3
    #pragma HLS BIND_STORAGE variable=val_buf type=ram_t2p impl=bram

    // 3 copies of penalty for parallel read (2 reads per copy via ram_2p)
    penalty_t pen_buf_0[TILE_H_H][TILE_W_H];
    penalty_t pen_buf_1[TILE_H_H][TILE_W_H];
    penalty_t pen_buf_2[TILE_H_H][TILE_W_H];
    #pragma HLS BIND_STORAGE variable=pen_buf_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_1 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_2 type=ram_2p impl=bram

    // -----------------------------------------------------------------------
    // Tile loop: sequential load -> compute -> store
    // -----------------------------------------------------------------------
    value_t global_max_delta = 0;

    TILE_Y: for (int ty = 0; ty < num_tiles_y; ty++) {
        TILE_X: for (int tx = 0; tx < num_tiles_x; tx++) {
            // Checkerboard assignment: process only tiles where (tx+ty)%2 == cu_id
            if ((tx + ty) % 2 != cu_id) continue;

            int tile_ox = tx * TILE_W;
            int tile_oy = ty * TILE_H;

            // Actual tile dimensions (handle map boundary)
            int tile_w = TILE_W;
            if (tile_ox + TILE_W > map_x) tile_w = map_x - tile_ox;
            int tile_h = TILE_H;
            if (tile_oy + TILE_H > map_y) tile_h = map_y - tile_oy;

            // Load tile + halo from DDR
            load_tile(value_table, penalty_table,
                      val_buf, pen_buf_0, pen_buf_1, pen_buf_2,
                      tile_ox, tile_oy, map_x, map_y);

            // Bellman update
            value_t tile_delta;
            compute_bellman(val_buf, pen_buf_0, pen_buf_1, pen_buf_2,
                           delta_table, tile_w, tile_h, tile_delta);

            if (tile_delta > global_max_delta)
                global_max_delta = tile_delta;

            // Store updated tile back to DDR
            store_tile(value_table, val_buf,
                       tile_ox, tile_oy, tile_w, tile_h, map_x);
        }
    }

    *max_delta = global_max_delta;
}
