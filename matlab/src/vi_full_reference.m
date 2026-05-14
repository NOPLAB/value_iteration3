function [value_table, action_table, sweeps, final_delta] = vi_full_reference( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, threshold, max_sweeps)
%VI_FULL_REFERENCE Paper-aligned brute-force value iteration reference.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    PB = double(p.PROB_BASE);
    NT = p.N_THETA;
    NA = p.N_ACTIONS;
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
                    best = best_action_cost(value_table, penalty_table, ...
                        trans_model, ix, iy, it, map_x, map_y, MV, PB, NT, NA);
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

function best = best_action_cost(value_table, penalty_table, trans_model, ...
    ix, iy, it, map_x, map_y, MV, PB, NT, NA)

    best = MV;
    for a = 1:NA
        c = action_cost(value_table, penalty_table, trans_model, ...
            ix, iy, it, map_x, map_y, a, MV, PB, NT);
        if c < best
            best = c;
        end
    end
end

function c = action_cost(value_table, penalty_table, trans_model, ...
    ix, iy, it, map_x, map_y, a, MV, PB, NT)

    accum = 0;
    n_out = trans_model.n_outcomes(a, it);
    for k = 1:n_out
        nx = ix + trans_model.dix(a, it, k);
        ny = iy + trans_model.diy(a, it, k);
        nt = it + trans_model.dit(a, it, k);

        if nt < 1
            nt = nt + NT;
        elseif nt > NT
            nt = nt - NT;
        end

        if nx < 1 || nx > map_x || ny < 1 || ny > map_y
            c = MV;
            return;
        end

        step_cost = cost_of(value_table(ny, nx, nt), penalty_table(ny, nx));
        if step_cost == MV
            c = MV;
            return;
        end

        accum = accum + step_cost * trans_model.prob(a, it, k);
    end

    c = floor(accum / PB);
    if c >= MV
        c = MV - 1;
    end
end
