function bb = bb_alloc3d(map_x, map_y, n_theta)
%BB_ALLOC3D Zero-initialized 3D bitboard sized [map_y, words_per_row, n_theta].
    bb = zeros(map_y, bb_words_per_row(map_x), n_theta, 'uint64');
end
