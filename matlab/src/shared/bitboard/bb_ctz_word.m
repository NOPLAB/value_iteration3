function n = bb_ctz_word(w)
%BB_CTZ_WORD Count trailing zeros of a uint64 word. Returns 64 for w == 0.
    if w == uint64(0)
        n = 64;
        return;
    end
    n = 0;
    if bitand(w, uint64(4294967295)) == 0
        n = n + 32; w = bitshift(w, -32);
    end
    if bitand(w, uint64(65535)) == 0
        n = n + 16; w = bitshift(w, -16);
    end
    if bitand(w, uint64(255)) == 0
        n = n + 8;  w = bitshift(w, -8);
    end
    if bitand(w, uint64(15)) == 0
        n = n + 4;  w = bitshift(w, -4);
    end
    if bitand(w, uint64(3)) == 0
        n = n + 2;  w = bitshift(w, -2);
    end
    if bitand(w, uint64(1)) == 0
        n = n + 1;
    end
end
