function out = bb_dilate3d(bb, map_x, dx, dy, dt)
%BB_DILATE3D 3D box dilation with periodic theta axis (modulo n_theta).
    [~, ~, n_theta] = size(bb);

    temp = zeros(size(bb), 'uint64');
    for it = 1:n_theta
        temp(:, :, it) = bb_dilate2d(bb(:, :, it), map_x, dx, dy);
    end

    if dt == 0
        out = temp;
        return;
    end

    out = temp;
    for st = 1:dt
        idx_plus  = mod((1:n_theta) - st - 1, n_theta) + 1;
        idx_minus = mod((1:n_theta) + st - 1, n_theta) + 1;
        out = bitor(out, temp(:, :, idx_plus));
        out = bitor(out, temp(:, :, idx_minus));
    end
end
