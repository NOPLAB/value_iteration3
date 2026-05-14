function [value_table, iters, total_updates] = vi_frontier_stack( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, max_iters)
%VI_FRONTIER_STACK Frontier-tracking VI with 60 stacked 2D bitboards.
%   Variant 3: each theta layer is its own 2D bitboard (cell-array storage).
%   expand() applies per-layer 2D dilation, then OR's neighboring theta layers.
%   Inner walk drains one layer at a time.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    NT = p.N_THETA;
    trans_model = coerce_transition_model(transitions);

    [mx, my, mt] = vi_frontier_max_displacement(trans_model);

    value_table(goal_mask) = 0;

    passable_bb = bb_from_logical2d(penalty_table ~= OB, map_x, map_y);

    goal_layers = cell(NT, 1);
    frontier    = cell(NT, 1);
    for it = 1:NT
        goal_layers{it} = bb_from_logical2d(goal_mask(:, :, it), map_x, map_y);
        frontier{it}    = bb_from_logical2d(value_table(:, :, it) < MV, map_x, map_y);
    end

    total_updates = 0;
    iters = 0;

    while stack_popcount(frontier) > 0 && iters < max_iters
        iters = iters + 1;

        dilated_self = cell(NT, 1);
        for it = 1:NT
            dilated_self{it} = bb_dilate2d(frontier{it}, map_x, mx, my);
        end

        candidates   = cell(NT, 1);
        new_frontier = cell(NT, 1);
        for it = 1:NT
            cand = dilated_self{it};
            for st = 1:mt
                it_minus = mod(it - st - 1, NT) + 1;
                it_plus  = mod(it + st - 1, NT) + 1;
                cand = bitor(cand, dilated_self{it_minus});
                cand = bitor(cand, dilated_self{it_plus});
            end
            cand = bitand(cand, passable_bb);
            cand = bitand(cand, bitcmp(goal_layers{it}));
            candidates{it}   = cand;
            new_frontier{it} = bb_alloc2d(map_x, map_y);
        end

        for it = 1:NT
            pts = bb_enumerate2d(candidates{it}, map_x, map_y);
            for n = 1:size(pts, 1)
                ix = pts(n, 1);
                iy = pts(n, 2);
                old_val = value_table(iy, ix, it);
                v_new = vi_frontier_bellman(value_table, penalty_table, ...
                    trans_model, ix, iy, it, map_x, map_y);
                if v_new < old_val
                    value_table(iy, ix, it) = v_new;
                    total_updates = total_updates + 1;
                    new_frontier{it} = bb_set2d(new_frontier{it}, ix, iy);
                end
            end
        end

        frontier = new_frontier;
    end
end

function n = stack_popcount(frontier_cell)
    n = 0;
    for it = 1:numel(frontier_cell)
        n = n + bb_popcount(frontier_cell{it});
    end
end
