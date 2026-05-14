function mask = bb_row_mask(map_x)
%BB_ROW_MASK Per-word mask isolating valid bits within a row.
%   Returns a 1 x words_per_row uint64 vector. Every word is all-ones except
%   the last word, which clears bit positions >= mod(map_x, 64).
    nw = bb_words_per_row(map_x);
    mask = repmat(intmax('uint64'), 1, nw);
    rem_bits = mod(map_x, 64);
    if rem_bits ~= 0
        mask(end) = bitshift(uint64(1), rem_bits) - uint64(1);
    end
end
