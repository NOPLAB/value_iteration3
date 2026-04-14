function action_table = compute_action_table_reference(value_table, penalty_table, ...
    goal_mask, transitions, map_x, map_y)
%COMPUTE_ACTION_TABLE_REFERENCE Compute argmin action using paper semantics.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    trans_model = coerce_transition_model(transitions);
    action_table = zeros(map_y, map_x, p.N_THETA, 'uint8');

    for iy = 1:map_y
        for ix = 1:map_x
            for it = 1:p.N_THETA
                if goal_mask(iy, ix, it) || penalty_table(iy, ix) == OB
                    action_table(iy, ix, it) = uint8(0);
                    continue;
                end

                best_cost = MV;
                best_act = 0;
                for a = 1:p.N_ACTIONS
                    c = action_cost(value_table, penalty_table, trans_model, ...
                        ix, iy, it, map_x, map_y, a);
                    if c < best_cost
                        best_cost = c;
                        best_act = a - 1;
                    end
                end
                action_table(iy, ix, it) = uint8(best_act);
            end
        end
    end
end

function c = action_cost(value_table, penalty_table, trans_model, ix, iy, it, map_x, map_y, a)
    p = vi_params();
    MV = double(p.MAX_VALUE);

    accum = 0;
    n_out = trans_model.n_outcomes(a, it);
    for k = 1:n_out
        nx = ix + trans_model.dix(a, it, k);
        ny = iy + trans_model.diy(a, it, k);
        nt = it + trans_model.dit(a, it, k);

        if nt < 1
            nt = nt + p.N_THETA;
        elseif nt > p.N_THETA
            nt = nt - p.N_THETA;
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

    c = floor(accum / p.PROB_BASE);
    if c >= MV
        c = MV - 1;
    end
end
