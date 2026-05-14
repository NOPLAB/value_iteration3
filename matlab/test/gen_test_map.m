function [value, penalty, goal_x, goal_y, goal_mask, spec] = gen_test_map(map_x, map_y, map_type, opts)
%GEN_TEST_MAP Generate paper-aligned test maps for VI kernel testing.
%   Returns:
%     value     - double [map_y, map_x, N_THETA], initialized to MAX_VALUE
%     penalty   - double [map_y, map_x], 0=free, OBSTACLE=0xFFFF
%     goal_x/y  - 1-indexed goal-center cell
%     goal_mask - logical [map_y, map_x, N_THETA]
%     spec      - goal/map metadata used by make_goal_mask()
%
%   For map_type = 'random', opts.density (default 0.15) and opts.seed
%   (default 42) control the obstacle field. A 3x3 region around the goal is
%   always kept free so make_goal_mask() has somewhere to anchor.

    if nargin < 4
        opts = struct();
    end

    p = vi_params();
    MAX_VALUE        = double(p.MAX_VALUE);
    PENALTY_OBSTACLE = double(p.PENALTY_OBSTACLE);

    value   = MAX_VALUE * ones(map_y, map_x, p.N_THETA);
    penalty = zeros(map_y, map_x);

    goal_x = ceil(map_x / 2);
    goal_y = ceil(map_y / 2);

    switch map_type
        case 'empty'
            % Nothing else to do.

        case 'obstacle'
            wall_y = max(1, goal_y - 3);
            for wy = wall_y:min(map_y, wall_y + 1)
                for wx = max(1, goal_x - 3):min(map_x, goal_x + 3)
                    penalty(wy, wx) = PENALTY_OBSTACLE;
                end
            end

        case 'sentinel'
            if goal_y > 1
                penalty(goal_y - 1, goal_x) = PENALTY_OBSTACLE;
            end
            if goal_y < map_y
                penalty(goal_y + 1, goal_x) = PENALTY_OBSTACLE;
            end
            if goal_x > 1
                penalty(goal_y, goal_x - 1) = PENALTY_OBSTACLE;
            end

        case 'random'
            if ~isfield(opts, 'density'); opts.density = 0.15; end
            if ~isfield(opts, 'seed');    opts.seed    = 42;   end
            rng(opts.seed);
            mask = rand(map_y, map_x) < opts.density;
            keep_y = max(1, goal_y - 1):min(map_y, goal_y + 1);
            keep_x = max(1, goal_x - 1):min(map_x, goal_x + 1);
            mask(keep_y, keep_x) = false;
            penalty(mask) = PENALTY_OBSTACLE;

        otherwise
            error('Unknown map_type: %s', map_type);
    end

    spec = struct( ...
        'xy_resolution', 0.05, ...
        'map_origin_x', 0.0, ...
        'map_origin_y', 0.0, ...
        'goal_x', (goal_x - 0.5) * 0.05, ...
        'goal_y', (goal_y - 0.5) * 0.05, ...
        'goal_theta_deg', 90, ...
        'goal_radius_m', 0.30, ...
        'goal_margin_theta_deg', 15);

    goal_mask = make_goal_mask(map_x, map_y, spec);
    value(goal_mask) = 0;
end
