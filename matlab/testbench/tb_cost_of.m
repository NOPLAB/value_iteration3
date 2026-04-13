function tb_cost_of()
%TB_COST_OF Unit tests for cost_of function.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();

    MAX_VALUE        = double(p.MAX_VALUE);
    PENALTY_OBSTACLE = double(p.PENALTY_OBSTACLE);
    PENALTY_GOAL     = double(p.PENALTY_GOAL);

    % Test 1: Normal addition
    assert(cost_of(100, 50) == 150, 'Normal addition failed');

    % Test 2: Neighbor is MAX_VALUE (unreachable) → MAX_VALUE
    assert(cost_of(MAX_VALUE, 50) == MAX_VALUE, 'MAX_VALUE neighbor failed');

    % Test 3: Neighbor penalty is OBSTACLE → MAX_VALUE
    assert(cost_of(100, PENALTY_OBSTACLE) == MAX_VALUE, 'OBSTACLE penalty failed');

    % Test 4: Neighbor penalty is GOAL → treated as 0
    assert(cost_of(100, PENALTY_GOAL) == 100, 'GOAL penalty failed');

    % Test 5: Sum saturates at MAX_VALUE-1
    assert(cost_of(65000, 600) == MAX_VALUE - 1, 'Saturation failed');

    % Test 6: Goal cell with value 0 and GOAL penalty neighbor
    assert(cost_of(0, PENALTY_GOAL) == 0, 'Goal zero + GOAL penalty failed');

    % Test 7: Both MAX_VALUE
    assert(cost_of(MAX_VALUE, PENALTY_OBSTACLE) == MAX_VALUE, 'Both sentinel failed');

    disp('tb_cost_of: ALL PASSED');
end
