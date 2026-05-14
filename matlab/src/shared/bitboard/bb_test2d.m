function b = bb_test2d(bb, ix, iy)
%BB_TEST2D Return true if the bit at (ix, iy) is set.
    wi = floor((ix - 1) / 64) + 1;
    bi = mod(ix - 1, 64);
    b = bitand(bb(iy, wi), bitshift(uint64(1), bi)) ~= 0;
end
