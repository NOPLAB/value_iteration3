function pts = bb_enumerate2d(bb, map_x, map_y)
%BB_ENUMERATE2D Return all set bits as an N x 2 array of [ix, iy] (1-indexed).
%   Walks word-by-word using ctz; cost is O(popcount(bb) + map_y * nw).
    n = bb_popcount(bb);
    pts = zeros(n, 2);
    if n == 0
        return;
    end
    k = 0;
    nw = size(bb, 2);
    for iy = 1:map_y
        for wi = 1:nw
            w = bb(iy, wi);
            base = (wi - 1) * 64;
            while w ~= 0
                b = bb_ctz_word(w);
                ix = base + b + 1;
                if ix <= map_x
                    k = k + 1;
                    pts(k, 1) = ix;
                    pts(k, 2) = iy;
                end
                w = bitand(w, w - uint64(1));
            end
        end
    end
    pts = pts(1:k, :);
end
