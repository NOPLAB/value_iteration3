function bb = bb_from_logical2d(mask, map_x, map_y)
%BB_FROM_LOGICAL2D Convert a [map_y, map_x] logical mask into a 2D bitboard.
    bb = bb_alloc2d(map_x, map_y);
    [iys, ixs] = find(mask);
    for k = 1:numel(iys)
        bb = bb_set2d(bb, ixs(k), iys(k));
    end
end
