function out = bb_dilate2d(bb, map_x, dx, dy)
%BB_DILATE2D L-infinity box dilation of a 2D bitboard by (dx, dy).
%   Separable: Y dilation then X dilation. Result is masked to valid bits.
    [map_y, nw] = size(bb);
    if dx >= 64
        error('bb_dilate2d: dx >= 64 (%d) not supported', dx);
    end
    row_mask_vec = bb_row_mask(map_x);
    row_mask_full = repmat(row_mask_vec, map_y, 1);

    y_dilated = bb;
    for sy = 1:dy
        y_dilated(sy+1:end, :) = bitor(y_dilated(sy+1:end, :), bb(1:end-sy, :));
        y_dilated(1:end-sy, :) = bitor(y_dilated(1:end-sy, :), bb(sy+1:end, :));
    end

    out = y_dilated;
    if dx > 0
        for sx_abs = 1:dx
            for iy = 1:map_y
                row_orig = y_dilated(iy, :);
                row_pos = bb_shift_row(row_orig,  sx_abs, row_mask_vec);
                row_neg = bb_shift_row(row_orig, -sx_abs, row_mask_vec);
                out(iy, :) = bitor(out(iy, :), bitor(row_pos, row_neg));
            end
        end
    end

    out = bitand(out, row_mask_full);
end
