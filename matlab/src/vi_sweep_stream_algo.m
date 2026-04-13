function [value_table, max_delta] = vi_sweep_stream_algo(value_table, ...
    value_table_rd, penalty_table, trans, map_x, map_y, cu_id)
%VI_SWEEP_STREAM_ALGO Top-level streaming VI kernel.
%   Matches fpga/hls/stream/src/vi_sweep_stream_top.cpp.
%   One call = one CU's sweep. Call with cu_id=0 then cu_id=1 for a full sweep.
%
%   value_table    — [map_y, map_x, N_THETA] double (R/W)
%   value_table_rd — [map_y, map_x, N_THETA] double (R)
%   penalty_table  — [map_y, map_x] double
%   trans          — uint32 [360 x 1] packed transition table
%   map_x, map_y   — map dimensions
%   cu_id          — 0=forward, 1=reverse

    p = vi_params();

    % 1. Unpack transition table
    delta_table = zeros(p.N_ACTIONS, p.N_THETA, 3);
    for i = 1:p.TRANS_TABLE_SIZE
        a = floor((i-1) / p.N_THETA) + 1;
        t = mod(i-1, p.N_THETA) + 1;
        w = trans(i);
        delta_table(a, t, 1) = double(typecast(uint8(bitand(w, 255)), 'int8'));
        delta_table(a, t, 2) = double(typecast(uint8(bitand(bitshift(w,-8), 255)), 'int8'));
        delta_table(a, t, 3) = double(typecast(uint8(bitand(bitshift(w,-16), 255)), 'int8'));
    end

    % 2. Compute strip layout
    num_strips = ceil(map_x / p.STRIP_W_MAX);
    half_strips = ceil(num_strips / 2);

    global_max_delta = 0;

    % 3. Iterate X-strips
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
            value_table_rd, penalty_table, delta_table, ...
            map_x, map_y, strip_x0, strip_w, cu_id);

        if strip_delta > global_max_delta
            global_max_delta = strip_delta;
        end
    end

    max_delta = global_max_delta;
end
