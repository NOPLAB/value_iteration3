function [value_table, action_table, sweeps, final_delta] = vi_full_reference( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, threshold, max_sweeps)
%VI_FULL_REFERENCE Paper-aligned brute-force value iteration reference.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    NT = p.N_THETA;
    trans_model = coerce_transition_model(transitions);

    final_delta = MV;
    sweeps = 0;

    for sweep = 1:max_sweeps
        max_delta = 0;
        for iy = 1:map_y
            for ix = 1:map_x
                if penalty_table(iy, ix) == OB
                    continue;
                end

                for it = 1:NT
                    if goal_mask(iy, ix, it)
                        value_table(iy, ix, it) = 0;
                        continue;
                    end

                    old_val = value_table(iy, ix, it);
                    best = vi_frontier_bellman(value_table, penalty_table, ...
                        trans_model, ix, iy, it, map_x, map_y);
                    value_table(iy, ix, it) = best;

                    d = abs(best - old_val);
                    if d > max_delta
                        max_delta = d;
                    end
                end
            end
        end

        sweeps = sweep;
        final_delta = max_delta;
        if max_delta <= threshold
            break;
        end
    end

    value_table(goal_mask) = 0;
    action_table = compute_action_table_reference(value_table, penalty_table, ...
        goal_mask, trans_model, map_x, map_y);
end
