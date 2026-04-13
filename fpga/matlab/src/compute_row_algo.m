function [val_buf, row_max_delta] = compute_row_algo(val_buf, pen_buf, ...
                                                      delta_table, win_center, ...
                                                      strip_w, cu_id)
%COMPUTE_ROW_ALGO Bellman update for one row in the sliding window.
%   Matches fpga/hls/stream/src/compute_row.cpp.
%   All arithmetic in double (Phase A).
%
%   val_buf      — [WINDOW_ROWS, BUF_W, N_THETA] double (modified in-place)
%   pen_buf      — [WINDOW_ROWS, BUF_W] double
%   delta_table  — [N_ACTIONS, N_THETA, 3] double (dix, diy, dit)
%   win_center   — 1-indexed row in circular buffer
%   strip_w      — active strip width
%   cu_id        — 0=forward, 1=reverse

    p = vi_params();
    MV = double(p.MAX_VALUE);
    GOAL = double(p.PENALTY_GOAL);

    local_max = 0;

    % Precompute ny lookup (1-indexed)
    y_sign = 1;
    if cu_id == 1
        y_sign = -1;
    end
    ny_lut = zeros(p.N_ACTIONS, p.N_THETA);
    for a = 1:p.N_ACTIONS
        for it = 1:p.N_THETA
            diy = y_sign * delta_table(a, it, 2);
            ny = win_center + diy;
            % Circular wrap (1-indexed)
            if ny < 1
                ny = ny + p.WINDOW_ROWS;
            elseif ny > p.WINDOW_ROWS
                ny = ny - p.WINDOW_ROWS;
            end
            ny_lut(a, it) = ny;
        end
    end

    % X loop
    for ix_raw = 0:strip_w-1
        if cu_id == 0
            ix = ix_raw;
        else
            ix = strip_w - 1 - ix_raw;
        end
        bx = ix + p.HALO_MAX + 1;  % 1-indexed

        cell_pen = pen_buf(win_center, bx);
        skip = (cell_pen >= GOAL);

        % Theta loop
        for it = 1:p.N_THETA
            old_val = val_buf(win_center, bx, it);

            if skip
                continue;
            end

            % Theta wrapping for turn actions
            it_l = it + 3;
            if it_l > p.N_THETA, it_l = it_l - p.N_THETA; end
            it_r = it - 3;
            if it_r < 1, it_r = it_r + p.N_THETA; end

            % Action 0: forward (same theta)
            nx0 = bx + delta_table(1, it, 1);
            c0 = cost_of(val_buf(ny_lut(1,it), nx0, it), ...
                         pen_buf(ny_lut(1,it), nx0));

            % Action 1: backward (same theta)
            nx1 = bx + delta_table(2, it, 1);
            c1 = cost_of(val_buf(ny_lut(2,it), nx1, it), ...
                         pen_buf(ny_lut(2,it), nx1));

            % Action 2: left (theta + 3)
            nx2 = bx + delta_table(3, it, 1);
            c2 = cost_of(val_buf(ny_lut(3,it), nx2, it_l), ...
                         pen_buf(ny_lut(3,it), nx2));

            % Action 3: right (theta - 3)
            nx3 = bx + delta_table(4, it, 1);
            c3 = cost_of(val_buf(ny_lut(4,it), nx3, it_r), ...
                         pen_buf(ny_lut(4,it), nx3));

            % Action 4: fwd-left (theta + 3)
            nx4 = bx + delta_table(5, it, 1);
            c4 = cost_of(val_buf(ny_lut(5,it), nx4, it_l), ...
                         pen_buf(ny_lut(5,it), nx4));

            % Action 5: fwd-right (theta - 3)
            nx5 = bx + delta_table(6, it, 1);
            c5 = cost_of(val_buf(ny_lut(6,it), nx5, it_r), ...
                         pen_buf(ny_lut(6,it), nx5));

            % Min-reduction tree
            min01 = min(c0, c1);
            min23 = min(c2, c3);
            min45 = min(c4, c5);
            min03 = min(min01, min23);
            min_cost = min(min03, min45);

            val_buf(win_center, bx, it) = min_cost;

            d = abs(min_cost - old_val);
            if d > local_max
                local_max = d;
            end
        end
    end

    row_max_delta = local_max;
end
