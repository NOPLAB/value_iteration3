function bb = bb_from_logical3d(mask, map_x, map_y, n_theta)
%BB_FROM_LOGICAL3D Convert a [map_y, map_x, n_theta] logical mask into a 3D bitboard.
    if nargin < 2
        map_y   = size(mask, 1);
        map_x   = size(mask, 2);
        n_theta = size(mask, 3);
    end
    bb = bb_alloc3d(map_x, map_y, n_theta);
    for it = 1:n_theta
        layer = mask(:, :, it);
        if ~any(layer(:))
            continue;
        end
        [iys, ixs] = find(layer);
        for k = 1:numel(iys)
            bb = bb_set3d(bb, ixs(k), iys(k), it);
        end
    end
end
