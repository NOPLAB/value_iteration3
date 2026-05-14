function [value_table, iters, total_updates] = vi_frontier_2d( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, max_iters)
%VI_FRONTIER_2D Frontier-tracking VI with a 2D (spatial) bitboard.
%   Variant 1: when a spatial cell enters the frontier, all N_THETA layers are
%   re-evaluated. The Bellman backup is bit-identical to vi_full_reference, so
%   converged outputs match exactly.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    NT = p.N_THETA;
    trans_model = coerce_transition_model(transitions);

    [mx, my, ~] = vi_frontier_max_displacement(trans_model);

    value_table(goal_mask) = 0;

    passable_bb = bb_from_logical2d(penalty_table ~= OB, map_x, map_y);

    seed_logical = any(value_table < MV, 3);
    frontier = bb_from_logical2d(seed_logical, map_x, map_y);

    total_updates = 0;
    iters = 0;

    while bb_popcount(frontier) > 0 && iters < max_iters
        iters = iters + 1;

        candidates = bitand(bb_dilate2d(frontier, map_x, mx, my), passable_bb);

        pts = bb_enumerate2d(candidates, map_x, map_y);
        new_frontier = bb_alloc2d(map_x, map_y);

        for n = 1:size(pts, 1)
            ix = pts(n, 1);
            iy = pts(n, 2);
            changed = false;
            for it = 1:NT
                if goal_mask(iy, ix, it)
                    continue;
                end
                old_val = value_table(iy, ix, it);
                v_new = vi_frontier_bellman(value_table, penalty_table, ...
                    trans_model, ix, iy, it, map_x, map_y);
                if v_new < old_val
                    value_table(iy, ix, it) = v_new;
                    total_updates = total_updates + 1;
                    changed = true;
                end
            end
            if changed
                new_frontier = bb_set2d(new_frontier, ix, iy);
            end
        end

        frontier = new_frontier;
    end
end
