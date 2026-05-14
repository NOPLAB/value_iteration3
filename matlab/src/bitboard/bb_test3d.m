function b = bb_test3d(bb, ix, iy, it)
%BB_TEST3D Return true if the bit at (ix, iy, it) is set.
    wi = floor((ix - 1) / 64) + 1;
    bi = mod(ix - 1, 64);
    b = bitand(bb(iy, wi, it), bitshift(uint64(1), bi)) ~= 0;
end
