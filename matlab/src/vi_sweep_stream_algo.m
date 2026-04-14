function [value_table, max_delta] = vi_sweep_stream_algo(value_table, ...
    value_table_rd, penalty_table, goal_mask, trans, map_x, map_y, cu_id)
%VI_SWEEP_STREAM_ALGO Top-level streaming VI kernel.
%   One call = one CU's sweep. Call with cu_id=0 then cu_id=1 for a full sweep.

    p = vi_params();
    trans_model = coerce_transition_model(trans);

    num_strips = ceil(map_x / p.STRIP_W_MAX);
    half_strips = ceil(num_strips / 2);
    global_max_delta = 0;

    for si = 0:half_strips-1
        if cu_id == 0
            sx = si;
        else
            sx = num_strips - 1 - si;
        end
        if sx < 0 || sx >= num_strips
            break;
        end

        strip_x0 = sx * p.STRIP_W_MAX;
        strip_w = min(p.STRIP_W_MAX, map_x - strip_x0);

        [value_table, strip_delta] = stream_strip_algo(value_table, ...
            value_table_rd, penalty_table, goal_mask, trans_model, ...
            map_x, map_y, strip_x0, strip_w, cu_id);

        if strip_delta > global_max_delta
            global_max_delta = strip_delta;
        end
    end

    value_table(goal_mask) = 0;

    max_delta = global_max_delta;
end
