function [value_table, strip_max_delta] = stream_strip_algo(value_table, ...
    value_table_rd, penalty_table, goal_mask, trans_model, map_x, map_y, ...
    strip_x0, strip_w, cu_id)
%STREAM_STRIP_ALGO Process one X-strip with sliding window.
%   Paper-aligned streaming Bellman sweep.

    p = vi_params();
    local_max = 0;
    trans_model = coerce_transition_model(trans_model);

    val_buf = zeros(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);
    pen_buf = zeros(p.WINDOW_ROWS, p.BUF_W);
    goal_buf = false(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);

    for wr = 0:p.WINDOW_ROWS-1
        if cu_id == 0
            gy = -p.HALO_MAX + wr;
        else
            gy = (map_y - 1) + p.HALO_MAX - wr;
        end
        slot = wr + 1;
        [val_buf(slot, :, :), pen_row, goal_row] = load_row_algo(value_table_rd, ...
            penalty_table, goal_mask, gy, strip_x0, strip_w, map_x, map_y);
        pen_buf(slot, :) = pen_row;
        goal_buf(slot, :, :) = goal_row;
    end

    for iy_raw = 0:map_y-1
        if cu_id == 0
            iy = iy_raw;
        else
            iy = map_y - 1 - iy_raw;
        end
        win_center = mod(iy_raw + p.HALO_MAX, p.WINDOW_ROWS) + 1;

        [val_buf, row_delta] = compute_row_algo(val_buf, pen_buf, goal_buf, ...
            trans_model, win_center, strip_w, cu_id);
        if row_delta > local_max
            local_max = row_delta;
        end

        value_table = store_row_algo(squeeze(val_buf(win_center, :, :)), ...
            value_table, iy, strip_x0, strip_w, map_x);

        evict_slot = mod(iy_raw, p.WINDOW_ROWS) + 1;
        if cu_id == 0
            next_gy = iy_raw + p.HALO_MAX + 1;
        else
            next_gy = (map_y - 1) - (iy_raw + p.HALO_MAX + 1);
        end
        [val_buf(evict_slot, :, :), pen_row, goal_row] = load_row_algo(value_table_rd, ...
            penalty_table, goal_mask, next_gy, strip_x0, strip_w, map_x, map_y);
        pen_buf(evict_slot, :) = pen_row;
        goal_buf(evict_slot, :, :) = goal_row;
    end

    strip_max_delta = local_max;
end
