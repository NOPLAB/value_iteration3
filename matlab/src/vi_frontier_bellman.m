function v_new = vi_frontier_bellman(value_table, penalty_table, trans_model, ...
    ix, iy, it, map_x, map_y)
%VI_FRONTIER_BELLMAN Bit-exact Bellman backup for a single (ix, iy, it) state.
%   Mirrors the action_cost / best_action_cost logic in vi_full_reference so
%   the frontier-VI variants converge to identical fixed points.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    PB = double(p.PROB_BASE);
    NT = p.N_THETA;
    NA = p.N_ACTIONS;

    v_new = MV;
    for a = 1:NA
        accum = 0;
        n_out = trans_model.n_outcomes(a, it);
        valid = true;
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
                accum = MV;
                valid = false;
                break;
            end

            step_cost = cost_of(value_table(ny, nx, nt), penalty_table(ny, nx));
            if step_cost == MV
                accum = MV;
                valid = false;
                break;
            end

            accum = accum + step_cost * trans_model.prob(a, it, k);
        end

        if valid
            c = floor(accum / PB);
            if c >= MV
                c = MV - 1;
            end
        else
            c = MV;
        end
        if c < v_new
            v_new = c;
        end
    end
end
