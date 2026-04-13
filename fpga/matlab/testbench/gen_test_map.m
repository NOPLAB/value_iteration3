function [value, penalty, goal_x, goal_y] = gen_test_map(map_x, map_y, map_type)
%GEN_TEST_MAP Generate test maps for VI kernel testing.
%   map_type:
%     'empty'    — no obstacles, goal at center
%     'obstacle' — rectangular obstacle block, goal at center
%     'sentinel' — GOAL cell surrounded by OBSTACLE on 3 sides
%
%   Returns:
%     value   — double [map_y, map_x, N_THETA], initialized to MAX_VALUE (goal=0)
%     penalty — double [map_y, map_x], 0=free, OBSTACLE=0xFFFF, GOAL=0xFFFE
%     goal_x, goal_y — 1-indexed goal position

    p = vi_params();
    MAX_VALUE        = double(p.MAX_VALUE);
    PENALTY_OBSTACLE = double(p.PENALTY_OBSTACLE);
    PENALTY_GOAL     = double(p.PENALTY_GOAL);

    value   = MAX_VALUE * ones(map_y, map_x, p.N_THETA);
    penalty = zeros(map_y, map_x);

    goal_x = ceil(map_x / 2);
    goal_y = ceil(map_y / 2);

    switch map_type
        case 'empty'
            % Nothing else to do

        case 'obstacle'
            % Place a 2-cell-thick wall above the goal
            wall_y = max(1, goal_y - 3);
            for wy = wall_y:min(map_y, wall_y+1)
                for wx = max(1, goal_x-3):min(map_x, goal_x+3)
                    penalty(wy, wx) = PENALTY_OBSTACLE;
                end
            end

        case 'sentinel'
            % Surround goal on 3 sides with obstacles (leave right side open)
            if goal_y > 1
                penalty(goal_y-1, goal_x) = PENALTY_OBSTACLE;
            end
            if goal_y < map_y
                penalty(goal_y+1, goal_x) = PENALTY_OBSTACLE;
            end
            if goal_x > 1
                penalty(goal_y, goal_x-1) = PENALTY_OBSTACLE;
            end

        otherwise
            error('Unknown map_type: %s', map_type);
    end

    % Set goal
    penalty(goal_y, goal_x) = PENALTY_GOAL;
    value(goal_y, goal_x, :) = 0;
end
