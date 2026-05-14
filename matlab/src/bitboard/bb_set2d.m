function bb = bb_set2d(bb, ix, iy)
%BB_SET2D Set the bit at (ix, iy) in a 2D bitboard.
    wi = floor((ix - 1) / 64) + 1;
    bi = mod(ix - 1, 64);
    bb(iy, wi) = bitor(bb(iy, wi), bitshift(uint64(1), bi));
end
