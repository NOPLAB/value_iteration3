function [val_row, pen_row] = load_row_algo(value_table, penalty_table, ...
                                             gy, strip_x0, strip_w, map_x, map_y)
%LOAD_ROW_ALGO Load one row with halo from value/penalty tables.
%   Matches fpga/hls/stream/src/load_store_row.cpp:load_row().
%   gy: 0-indexed global Y coordinate.
%   strip_x0: 0-indexed X start of strip.
%   All arrays are double (Phase A).
%
%   Returns:
%     val_row — [BUF_W, N_THETA] double
%     pen_row — [BUF_W, 1] double

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);

    buf_w = strip_w + 2 * p.HALO_MAX;

    % Phase A: Fill with sentinels
    val_row = MV * ones(p.BUF_W, p.N_THETA);
    pen_row = OB * ones(p.BUF_W, 1);

    % OOB check
    if gy < 0 || gy >= map_y
        return;
    end

    % Phase B: Compute in-bounds X range
    gx_start = strip_x0 - p.HALO_MAX;
    x0_global = max(0, gx_start);
    x1_global = min(map_x, gx_start + buf_w);
    x_count = x1_global - x0_global;
    lx_offset = x0_global - gx_start;  % 0-indexed local offset

    if x_count <= 0
        return;
    end

    % Phase C: Copy penalty (1-indexed MATLAB arrays)
    gy1 = gy + 1;  % 0-indexed -> 1-indexed
    for i = 0:x_count-1
        gx1 = x0_global + i + 1;
        lx1 = lx_offset + i + 1;
        pen_row(lx1) = penalty_table(gy1, gx1);
    end

    % Phase D: Copy value
    for i = 0:x_count-1
        gx1 = x0_global + i + 1;
        lx1 = lx_offset + i + 1;
        for it = 1:p.N_THETA
            val_row(lx1, it) = value_table(gy1, gx1, it);
        end
    end
end
