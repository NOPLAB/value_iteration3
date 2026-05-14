function mask = bb_to_logical3d(bb, map_x, map_y, n_theta)
%BB_TO_LOGICAL3D Convert a 3D bitboard back to a [map_y, map_x, n_theta] logical mask.
    mask = false(map_y, map_x, n_theta);
    nw = size(bb, 2);
    for it = 1:n_theta
        for iy = 1:map_y
            for wi = 1:nw
                w = bb(iy, wi, it);
                base = (wi - 1) * 64;
                while w ~= 0
                    b = bb_ctz_word(w);
                    ix = base + b + 1;
                    if ix <= map_x
                        mask(iy, ix, it) = true;
                    end
                    w = bitand(w, w - uint64(1));
                end
            end
        end
    end
end
