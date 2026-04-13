function value_table = store_row_algo(val_row, value_table, ...
                                       gy, strip_x0, strip_w, map_x)
%STORE_ROW_ALGO Store one row (inner cells, no halo) back to value table.
%   Matches fpga/hls/stream/src/load_store_row.cpp:store_row().
%   Modifies and returns value_table.

    p = vi_params();
    gy1 = gy + 1;  % 0-indexed -> 1-indexed

    for ix = 0:strip_w-1
        gx1 = strip_x0 + ix + 1;
        bx1 = ix + p.HALO_MAX + 1;  % skip halo, 1-indexed
        if gx1 >= 1 && gx1 <= map_x
            for it = 1:p.N_THETA
                value_table(gy1, gx1, it) = val_row(bx1, it);
            end
        end
    end
end
