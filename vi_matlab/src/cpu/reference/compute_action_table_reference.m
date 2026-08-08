function action_table = compute_action_table_reference(value_table, penalty_table, ...
    goal_mask, transitions, map_x, map_y)
%COMPUTE_ACTION_TABLE_REFERENCE Compute argmin action using paper semantics.

    p = vi_params();
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

                [~, best_act] = vi_frontier_bellman(value_table, penalty_table, ...
                    trans_model, ix, iy, it, map_x, map_y);
                action_table(iy, ix, it) = uint8(best_act);
            end
        end
    end
end
