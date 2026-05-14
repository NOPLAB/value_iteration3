function [value_table, iters, total_updates] = vi_frontier_3d( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, max_iters)
%VI_FRONTIER_3D Frontier-tracking VI with a 3D (x, y, theta) bitboard.
%   Variant 2: each (ix, iy, it) is an independent state in the frontier.
%   Predecessor cone is a 3D box with periodic theta.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    NT = p.N_THETA;
    trans_model = coerce_transition_model(transitions);

    [mx, my, mt] = vi_frontier_max_displacement(trans_model);

    value_table(goal_mask) = 0;

    passable_2d = bb_from_logical2d(penalty_table ~= OB, map_x, map_y);
    passable_bb = repmat(passable_2d, [1, 1, NT]);

    goal_bb = bb_from_logical3d(goal_mask, map_x, map_y, NT);
    not_goal_bb = bitcmp(goal_bb);  % flip all bits, will be masked by passable

    frontier = bb_from_logical3d(value_table < MV, map_x, map_y, NT);

    total_updates = 0;
    iters = 0;

    while bb_popcount(frontier) > 0 && iters < max_iters
        iters = iters + 1;

        candidates = bb_dilate3d(frontier, map_x, mx, my, mt);
        candidates = bitand(candidates, passable_bb);
        candidates = bitand(candidates, not_goal_bb);

        pts = bb_enumerate3d(candidates, map_x, map_y, NT);
        new_frontier = bb_alloc3d(map_x, map_y, NT);

        for n = 1:size(pts, 1)
            ix = pts(n, 1);
            iy = pts(n, 2);
            it = pts(n, 3);
            old_val = value_table(iy, ix, it);
            v_new = vi_frontier_bellman(value_table, penalty_table, ...
                trans_model, ix, iy, it, map_x, map_y);
            if v_new < old_val
                value_table(iy, ix, it) = v_new;
                total_updates = total_updates + 1;
                new_frontier = bb_set3d(new_frontier, ix, iy, it);
            end
        end

        frontier = new_frontier;
    end
end
