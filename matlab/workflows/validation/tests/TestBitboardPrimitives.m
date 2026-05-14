classdef TestBitboardPrimitives < TestBase
%TESTBITBOARDPRIMITIVES Low-level checks for matlab/src/shared/bitboard helpers.

    methods (Test)
        function testWordsPerRow(testCase)
            testCase.verifyEqual(bb_words_per_row(1),   1);
            testCase.verifyEqual(bb_words_per_row(64),  1);
            testCase.verifyEqual(bb_words_per_row(65),  2);
            testCase.verifyEqual(bb_words_per_row(128), 2);
            testCase.verifyEqual(bb_words_per_row(129), 3);
        end

        function testRowMaskTrailingBits(testCase)
            mask = bb_row_mask(8);
            testCase.verifyEqual(numel(mask), 1);
            testCase.verifyEqual(mask(1), uint64(255));

            mask = bb_row_mask(64);
            testCase.verifyEqual(mask(1), intmax('uint64'));

            mask = bb_row_mask(70);
            testCase.verifyEqual(numel(mask), 2);
            testCase.verifyEqual(mask(1), intmax('uint64'));
            testCase.verifyEqual(mask(2), uint64(63));  % bits 0..5
        end

        function testSetTestRoundtrip2d(testCase)
            bb = bb_alloc2d(16, 12);
            testCase.verifyEqual(bb_popcount(bb), 0);
            bb = bb_set2d(bb, 1, 1);
            bb = bb_set2d(bb, 16, 12);
            bb = bb_set2d(bb, 9, 6);
            testCase.verifyTrue(bb_test2d(bb, 1, 1));
            testCase.verifyTrue(bb_test2d(bb, 16, 12));
            testCase.verifyTrue(bb_test2d(bb, 9, 6));
            testCase.verifyFalse(bb_test2d(bb, 2, 1));
            testCase.verifyEqual(bb_popcount(bb), 3);
        end

        function testSetTestRoundtrip3d(testCase)
            bb = bb_alloc3d(8, 8, 60);
            bb = bb_set3d(bb, 1, 1, 1);
            bb = bb_set3d(bb, 8, 8, 60);
            bb = bb_set3d(bb, 4, 5, 30);
            testCase.verifyTrue(bb_test3d(bb, 1, 1, 1));
            testCase.verifyTrue(bb_test3d(bb, 8, 8, 60));
            testCase.verifyTrue(bb_test3d(bb, 4, 5, 30));
            testCase.verifyFalse(bb_test3d(bb, 4, 5, 31));
            testCase.verifyEqual(bb_popcount(bb), 3);
        end

        function testLogicalRoundtrip2d(testCase)
            rng(7);
            map_x = 33; map_y = 17;  % crosses 1-word boundary not, but mid-byte trailing
            mask = rand(map_y, map_x) < 0.3;
            bb = bb_from_logical2d(mask, map_x, map_y);
            round = bb_to_logical2d(bb, map_x, map_y);
            testCase.verifyEqual(round, mask);
        end

        function testLogicalRoundtrip3d(testCase)
            rng(13);
            map_x = 16; map_y = 16; n_theta = 8;
            mask = rand(map_y, map_x, n_theta) < 0.2;
            bb = bb_from_logical3d(mask, map_x, map_y, n_theta);
            round = bb_to_logical3d(bb, map_x, map_y, n_theta);
            testCase.verifyEqual(round, mask);
        end

        function testCtzCoversAll64Positions(testCase)
            for b = 0:63
                w = bitshift(uint64(1), b);
                testCase.verifyEqual(bb_ctz_word(w), b);
            end
            testCase.verifyEqual(bb_ctz_word(uint64(0)), 64);
            % Pattern with multiple bits: ctz returns lowest
            testCase.verifyEqual(bb_ctz_word(uint64(40)), 3);  % 0b101000 -> lowest at 3
        end

        function testEnumerate2dMatchesLogical(testCase)
            rng(21);
            map_x = 32; map_y = 16;
            mask = rand(map_y, map_x) < 0.1;
            bb = bb_from_logical2d(mask, map_x, map_y);
            pts = bb_enumerate2d(bb, map_x, map_y);
            % Build a logical from enumerated points and compare
            recon = false(map_y, map_x);
            for k = 1:size(pts, 1)
                recon(pts(k, 2), pts(k, 1)) = true;
            end
            testCase.verifyEqual(recon, mask);
            testCase.verifyEqual(size(pts, 1), nnz(mask));
        end

        function testDilate2dMatchesLogicalReference(testCase)
            rng(101);
            map_x = 17; map_y = 13;
            for dx = 0:2
                for dy = 0:2
                    mask = rand(map_y, map_x) < 0.15;
                    bb = bb_from_logical2d(mask, map_x, map_y);
                    dil = bb_dilate2d(bb, map_x, dx, dy);
                    actual = bb_to_logical2d(dil, map_x, map_y);
                    expected = local_dilate_logical2d(mask, dx, dy);
                    testCase.verifyEqual(actual, expected, ...
                        sprintf('dilate2d mismatch at dx=%d, dy=%d', dx, dy));
                end
            end
        end

        function testDilate3dThetaWrap(testCase)
            map_x = 8; map_y = 8; n_theta = 6;
            mask = false(map_y, map_x, n_theta);
            mask(4, 4, 1) = true;   % single bit at theta layer 1
            bb = bb_from_logical3d(mask, map_x, map_y, n_theta);

            dil = bb_dilate3d(bb, map_x, 0, 0, 1);
            actual = bb_to_logical3d(dil, map_x, map_y, n_theta);

            % theta=1 spreads to neighbors via wrap -> theta=n_theta and theta=2
            expected = false(map_y, map_x, n_theta);
            expected(4, 4, 1) = true;
            expected(4, 4, 2) = true;
            expected(4, 4, n_theta) = true;
            testCase.verifyEqual(actual, expected);
        end
    end
end

function out = local_dilate_logical2d(mask, dx, dy)
    [H, W] = size(mask);
    out = false(H, W);
    for sy = -dy:dy
        for sx = -dx:dx
            ys_src = max(1, 1 - sy):min(H, H - sy);
            xs_src = max(1, 1 - sx):min(W, W - sx);
            out(ys_src + sy, xs_src + sx) = ...
                out(ys_src + sy, xs_src + sx) | mask(ys_src, xs_src);
        end
    end
end
