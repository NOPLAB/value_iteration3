function [v_new, best_a] = vi_frontier_bellman(value_table, penalty_table, trans_model, ...
    ix, iy, it, map_x, map_y)
%VI_FRONTIER_BELLMAN Bit-exact Bellman backup for a single (ix, iy, it) state.
%   The single shared implementation of the paper's per-state backup; the
%   full-reference sweep and the frontier-VI variants all call it, so they
%   converge to identical fixed points.
%   Second output: 0-based argmin action (0 when no action beats MAX_VALUE).

    p = vi_params();
    MV = double(p.MAX_VALUE);
    PB = double(p.PROB_BASE);
    NT = p.N_THETA;
    NA = p.N_ACTIONS;

    v_new = MV;
    best_a = 0;
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
            best_a = a - 1;
        end
    end
end
