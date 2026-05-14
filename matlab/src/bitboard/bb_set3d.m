function bb = bb_set3d(bb, ix, iy, it)
%BB_SET3D Set the bit at (ix, iy, it) in a 3D bitboard.
    wi = floor((ix - 1) / 64) + 1;
    bi = mod(ix - 1, 64);
    bb(iy, wi, it) = bitor(bb(iy, wi, it), bitshift(uint64(1), bi));
end
