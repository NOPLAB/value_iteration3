function out = bb_shift_row(row, sx, row_mask)
%BB_SHIFT_ROW Horizontal bigint shift of one row by sx bit positions.
%   row, row_mask: 1 x nw uint64. sx in (-64, 64). Positive sx shifts toward
%   higher x (larger bit index). Out-of-range bits are masked off.
    nw = numel(row);
    out = zeros(1, nw, 'uint64');
    if sx == 0
        out = bitand(row, row_mask);
        return;
    end
    if sx > 0
        for i = 1:nw
            out(i) = bitshift(row(i), sx);
            if i > 1
                out(i) = bitor(out(i), bitshift(row(i-1), sx - 64));
            end
        end
    else
        n = -sx;
        for i = 1:nw
            out(i) = bitshift(row(i), -n);
            if i < nw
                out(i) = bitor(out(i), bitshift(row(i+1), 64 - n));
            end
        end
    end
    out = bitand(out, row_mask);
end
