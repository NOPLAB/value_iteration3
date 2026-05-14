function pts = bb_enumerate3d(bb, map_x, map_y, n_theta)
%BB_ENUMERATE3D Return all set bits as an N x 3 array of [ix, iy, it].
    n = bb_popcount(bb);
    pts = zeros(n, 3);
    if n == 0
        return;
    end
    k = 0;
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
                        k = k + 1;
                        pts(k, 1) = ix;
                        pts(k, 2) = iy;
                        pts(k, 3) = it;
                    end
                    w = bitand(w, w - uint64(1));
                end
            end
        end
    end
    pts = pts(1:k, :);
end
