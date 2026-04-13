function tb_load_store_row()
%TB_LOAD_STORE_ROW Unit tests for load_row_algo and store_row_algo.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);

    map_x = 16; map_y = 10;
    strip_x0 = 0; strip_w = 16;

    % Create known value and penalty tables (0-indexed coords internally)
    value_table = MV * ones(map_y, map_x, p.N_THETA);
    penalty_table = zeros(map_y, map_x);
    % Set some known values
    value_table(3, 5, :) = 100;  % (y=2 0-indexed, x=4 0-indexed)
    penalty_table(3, 5) = 42;

    % Test 1: Normal row load (gy=2, 0-indexed)
    [val_row, pen_row] = load_row_algo(value_table, penalty_table, ...
                                        2, strip_x0, strip_w, map_x, map_y);
    % val_row is [BUF_W, N_THETA], pen_row is [BUF_W, 1]
    % bx = x + HALO_MAX for in-strip cells
    bx = 4 + p.HALO_MAX + 1;  % +1 for MATLAB 1-indexing
    assert(val_row(bx, 1) == 100, 'Value not loaded correctly');
    assert(pen_row(bx) == 42, 'Penalty not loaded correctly');

    % Halo cells (x < 0) should be MAX_VALUE/OBSTACLE
    assert(val_row(1, 1) == MV, 'Left halo not sentinel');
    assert(pen_row(1) == OB, 'Left halo penalty not obstacle');

    % Test 2: Out-of-bounds row (gy = -1)
    [val_oob, pen_oob] = load_row_algo(value_table, penalty_table, ...
                                        -1, strip_x0, strip_w, map_x, map_y);
    assert(all(pen_oob == OB), 'OOB row penalty not all obstacle');
    assert(all(val_oob(:, 1) == MV), 'OOB row value not all max');

    % Test 3: Store and re-load round-trip
    val_row_modified = val_row;
    val_row_modified(p.HALO_MAX+1, 1) = 999;  % Modify first in-strip cell
    value_table2 = store_row_algo(val_row_modified, value_table, ...
                                   2, strip_x0, strip_w, map_x);
    % Verify written back
    assert(value_table2(3, 1, 1) == 999, 'Store did not write back');
    % Non-modified cells unchanged
    assert(value_table2(3, 5, 1) == 100, 'Store corrupted other cell');

    disp('tb_load_store_row: ALL PASSED');
end
