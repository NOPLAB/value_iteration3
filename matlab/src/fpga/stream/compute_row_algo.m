function [val_buf, row_max_delta] = compute_row_algo(val_buf, pen_buf, ...
                                                     goal_buf, trans_model, ...
                                                     win_center, strip_w, cu_id)
%COMPUTE_ROW_ALGO Bellman update for one row in the sliding window.
%   Paper-aligned cost model with Monte Carlo transition probabilities.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    trans_model = coerce_transition_model(trans_model);

    local_max = 0;

    for ix_raw = 0:strip_w-1
        if cu_id == 0
            ix = ix_raw;
        else
            ix = strip_w - 1 - ix_raw;
        end
        bx = ix + p.HALO_MAX + 1;

        cell_pen = pen_buf(win_center, bx);
        is_obstacle = (cell_pen == OB);

        for it = 1:p.N_THETA
            old_val = val_buf(win_center, bx, it);

            if goal_buf(win_center, bx, it)
                val_buf(win_center, bx, it) = 0;
                continue;
            end

            if is_obstacle
                continue;
            end

            best = MV;
            for a = 1:p.N_ACTIONS
                n_out = trans_model.n_outcomes(a, it);
                accum = 0;
                invalid = false;

                for k = 1:n_out
                    nx = bx + trans_model.dix(a, it, k);
                    if nx < 1 || nx > (strip_w + 2 * p.HALO_MAX)
                        invalid = true;
                        break;
                    end

                    ny = win_center + direction_signed(cu_id) * trans_model.diy(a, it, k);
                    if ny < 1
                        ny = ny + p.WINDOW_ROWS;
                    elseif ny > p.WINDOW_ROWS
                        ny = ny - p.WINDOW_ROWS;
                    end

                    nt = it + trans_model.dit(a, it, k);
                    if nt < 1
                        nt = nt + p.N_THETA;
                    elseif nt > p.N_THETA
                        nt = nt - p.N_THETA;
                    end

                    cost = cost_of(val_buf(ny, nx, nt), pen_buf(ny, nx));
                    if cost == MV
                        invalid = true;
                        break;
                    end

                    accum = accum + cost * trans_model.prob(a, it, k);
                end

                if invalid
                    act_cost = MV;
                else
                    act_cost = floor(accum / p.PROB_BASE);
                    if act_cost >= MV
                        act_cost = MV - 1;
                    end
                end

                if act_cost < best
                    best = act_cost;
                end
            end

            val_buf(win_center, bx, it) = best;
            d = abs(best - old_val);
            if d > local_max
                local_max = d;
            end
        end
    end

    row_max_delta = local_max;
end

function s = direction_signed(cu_id)
    if cu_id == 0
        s = 1;
    else
        s = -1;
    end
end
