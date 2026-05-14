function mask = bb_to_logical2d(bb, map_x, map_y)
%BB_TO_LOGICAL2D Convert a 2D bitboard back to a [map_y, map_x] logical mask.
    mask = false(map_y, map_x);
    nw = size(bb, 2);
    for iy = 1:map_y
        for wi = 1:nw
            w = bb(iy, wi);
            base = (wi - 1) * 64;
            while w ~= 0
                b = bb_ctz_word(w);
                ix = base + b + 1;
                if ix <= map_x
                    mask(iy, ix) = true;
                end
                w = bitand(w, w - uint64(1));
            end
        end
    end
end
