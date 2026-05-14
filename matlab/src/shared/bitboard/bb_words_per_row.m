function nw = bb_words_per_row(map_x)
%BB_WORDS_PER_ROW Number of uint64 words needed to hold map_x bits.
    nw = ceil(map_x / 64);
end
