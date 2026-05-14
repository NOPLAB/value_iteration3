function bb = bb_alloc2d(map_x, map_y)
%BB_ALLOC2D Zero-initialized 2D bitboard sized [map_y, words_per_row].
    bb = zeros(map_y, bb_words_per_row(map_x), 'uint64');
end
