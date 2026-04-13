function [value_table, strip_max_delta] = stream_strip_algo(value_table, ...
    value_table_rd, penalty_table, delta_table, map_x, map_y, ...
    strip_x0, strip_w, cu_id)
%STREAM_STRIP_ALGO Process one X-strip with sliding window.
%   Matches fpga/hls/stream/src/stream_strip.cpp.
%   All arithmetic in double (Phase A).
%
%   value_table    — [map_y, map_x, N_THETA] double (write destination)
%   value_table_rd — [map_y, map_x, N_THETA] double (read source)
%   penalty_table  — [map_y, map_x] double
%   delta_table    — [N_ACTIONS, N_THETA, 3] double
%   cu_id          — 0=forward (Y ascending), 1=reverse (Y descending)
%
%   Returns modified value_table and strip_max_delta.

    p = vi_params();
    local_max = 0;

    % Allocate line buffers: [WINDOW_ROWS, BUF_W, N_THETA] and [WINDOW_ROWS, BUF_W]
    val_buf = zeros(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);
    pen_buf = zeros(p.WINDOW_ROWS, p.BUF_W);

    % Initialize window: load WINDOW_ROWS rows
    for wr = 0:p.WINDOW_ROWS-1
        if cu_id == 0
            gy = -p.HALO_MAX + wr;
        else
            gy = (map_y - 1) + p.HALO_MAX - wr;
        end
        slot = wr + 1;  % 1-indexed
        [val_buf(slot,:,:), pen_row] = load_row_algo(value_table_rd, penalty_table, ...
                                                      gy, strip_x0, strip_w, map_x, map_y);
        pen_buf(slot, :) = pen_row;
    end

    % Stream through all rows
    for iy_raw = 0:map_y-1
        if cu_id == 0
            iy = iy_raw;
        else
            iy = map_y - 1 - iy_raw;
        end
        win_center = mod(iy_raw + p.HALO_MAX, p.WINDOW_ROWS) + 1;  % 1-indexed

        % Compute Bellman update
        [val_buf, row_delta] = compute_row_algo(val_buf, pen_buf, ...
                                                 delta_table, win_center, ...
                                                 strip_w, cu_id);
        if row_delta > local_max
            local_max = row_delta;
        end

        % Store updated row; squeeze [1, BUF_W, N_THETA] -> [BUF_W, N_THETA]
        value_table = store_row_algo(squeeze(val_buf(win_center,:,:)), value_table, ...
                                      iy, strip_x0, strip_w, map_x);

        % Evict oldest, load next
        evict_slot = mod(iy_raw, p.WINDOW_ROWS) + 1;  % 1-indexed
        if cu_id == 0
            next_gy = iy_raw + p.HALO_MAX + 1;
        else
            next_gy = (map_y - 1) - (iy_raw + p.HALO_MAX + 1);
        end
        [val_buf(evict_slot,:,:), pen_row] = load_row_algo(value_table_rd, penalty_table, ...
                                                            next_gy, strip_x0, strip_w, ...
                                                            map_x, map_y);
        pen_buf(evict_slot, :) = pen_row;
    end

    strip_max_delta = local_max;
end
