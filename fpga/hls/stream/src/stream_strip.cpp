#include "stream_strip.h"
#include "load_store_row.h"
#include "compute_row.h"

// DATAFLOW wrapper: store completed row (from store_buf via gmem0) while
// simultaneously prefetching next future row (via gmem2+gmem1 into val_buf).
// store_buf is read-only here; val/pen load targets are write-only.
// No shared arrays → DATAFLOW is legal.
static void store_and_prefetch(
    const value_t store_buf[BUF_W][N_THETA],
    value_t   val_load[BUF_W][N_THETA],
    penalty_t pen_load_0[BUF_W],
    penalty_t pen_load_1[BUF_W],
    penalty_t pen_load_2[BUF_W],
    value_t       *value_table,
    const value_t *value_table_rd,
    const penalty_t *penalty_table,
    int store_gy, int load_gy,
    int strip_x0, int strip_w,
    int map_x, int map_y)
{
    #pragma HLS INLINE off
    #pragma HLS DATAFLOW

    store_row(store_buf, value_table,
              store_gy, strip_x0, strip_w, map_x);

    load_row(val_load, pen_load_0, pen_load_1, pen_load_2,
             value_table_rd, penalty_table,
             load_gy, strip_x0, strip_w, map_x, map_y);
}

void stream_strip(
    value_t       *value_table,
    const value_t *value_table_rd,
    const penalty_t *penalty_table,
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int map_x, int map_y,
    int strip_x0,
    int strip_w,
    int cu_id,
    value_t &strip_max_delta)
{
    #pragma HLS INLINE off

    // --- Line buffer declaration (BRAM) ---
    value_t   val_buf[WINDOW_ROWS][BUF_W][N_THETA];
    penalty_t pen_buf_0[WINDOW_ROWS][BUF_W];
    penalty_t pen_buf_1[WINDOW_ROWS][BUF_W];
    penalty_t pen_buf_2[WINDOW_ROWS][BUF_W];

    #pragma HLS ARRAY_PARTITION variable=val_buf complete dim=3
    #pragma HLS BIND_STORAGE variable=val_buf type=ram_t2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_1 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_2 type=ram_2p impl=bram

    // Separate buffer for DDR writeback — decouples store path from val_buf
    // so store_row and load_row share no arrays, enabling DATAFLOW.
    value_t store_buf[BUF_W][N_THETA];
    #pragma HLS BIND_STORAGE variable=store_buf type=ram_2p impl=bram

    value_t local_max = 0;

    // --- Initialize window: load WINDOW_ROWS rows ---
    INIT_WIN: for (int wr = 0; wr < WINDOW_ROWS; wr++) {
        int gy;
        if (cu_id == 0)
            gy = -HALO_MAX + wr;
        else
            gy = (map_y - 1) + HALO_MAX - wr;

        load_row(val_buf[wr], pen_buf_0[wr], pen_buf_1[wr], pen_buf_2[wr],
                 value_table_rd, penalty_table,
                 gy, strip_x0, strip_w, map_x, map_y);
    }

    // --- Stream through all rows ---
    ROW_LOOP: for (int iy_raw = 0; iy_raw < map_y; iy_raw++) {
        #pragma HLS LOOP_TRIPCOUNT min=20 max=800
        int iy = (cu_id == 0) ? iy_raw : (map_y - 1 - iy_raw);
        int win_center = (iy_raw + HALO_MAX) % WINDOW_ROWS;

        // Compute Bellman update — writes to both val_buf and store_buf
        value_t row_delta;
        compute_row(val_buf, pen_buf_0, pen_buf_1, pen_buf_2,
                    delta_table, win_center, strip_w, cu_id, row_delta,
                    store_buf);
        if (row_delta > local_max) local_max = row_delta;

        // DATAFLOW: store completed row + prefetch next future row in parallel
        int evict_slot = iy_raw % WINDOW_ROWS;
        int next_gy;
        if (cu_id == 0)
            next_gy = iy_raw + HALO_MAX + 1;
        else
            next_gy = (map_y - 1) - (iy_raw + HALO_MAX + 1);

        store_and_prefetch(
            store_buf,
            val_buf[evict_slot],
            pen_buf_0[evict_slot], pen_buf_1[evict_slot], pen_buf_2[evict_slot],
            value_table, value_table_rd, penalty_table,
            iy, next_gy,
            strip_x0, strip_w, map_x, map_y);
    }

    strip_max_delta = local_max;
}
