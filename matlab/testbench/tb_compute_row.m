function tb_compute_row()
%TB_COMPUTE_ROW Unit tests for compute_row_algo.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    goal_buf = false(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);

    % Setup: 13-row window, BUF_W wide, N_THETA deep
    val_buf = MV * ones(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);
    pen_buf = OB * ones(p.WINDOW_ROWS, p.BUF_W);

    % Place a goal neighbor at (win_center, bx=10, theta=1) with value=0
    win_center = p.HALO_MAX + 1;  % 1-indexed center row (7)
    bx_goal = 10;
    val_buf(win_center, bx_goal, 1) = 0;
    pen_buf(win_center, bx_goal) = 0;
    goal_buf(win_center, bx_goal, 1) = true;

    % Free cell at bx=9 (reachable from bx_goal via action 1: dix=-1)
    pen_buf(win_center, 9) = 0;
    val_buf(win_center, 9, :) = MV;

    % Trivial delta_table: action 0 = dix+1, action 1 = dix-1
    delta_table = zeros(p.N_ACTIONS, p.N_THETA, 3);
    for it = 1:p.N_THETA
        delta_table(1, it, 1) = 1;   % action 0: dix=+1
        delta_table(2, it, 1) = -1;  % action 1: dix=-1
    end

    strip_w = 16;
    cu_id = 0;

    [val_buf_out, row_max_delta] = compute_row_algo(val_buf, pen_buf, ...
                                                     goal_buf, delta_table, ...
                                                     win_center, strip_w, cu_id);

    % Cell at bx=9 should now have value = cost_of(0, 0) = 1 because
    % the one-step base cost is added even when the next state is a goal.
    assert(val_buf_out(win_center, 9, 1) == 1, ...
        sprintf('Expected 1, got %d', val_buf_out(win_center, 9, 1)));

    % Goal state itself should be unchanged.
    assert(val_buf_out(win_center, bx_goal, 1) == 0, 'Goal cell modified');

    % Obstacle cells should be unchanged
    assert(val_buf_out(win_center, 1, 1) == MV, 'Obstacle cell changed');

    assert(row_max_delta >= 0, 'Negative delta');

    disp('tb_compute_row: ALL PASSED');
end
