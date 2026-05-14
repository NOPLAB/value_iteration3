function n = bb_popcount(bb)
%BB_POPCOUNT Total number of set bits across the bitboard.
    if isempty(bb)
        n = 0;
        return;
    end
    flat = bb(:);
    acc = uint64(0);
    for b = 1:64
        acc = acc + sum(bitget(flat, b));
    end
    n = double(acc);
end
