function [gmem2_rd_addr, gmem2_rd_len, gmem2_rd_avalid, gmem2_rd_dready, ...
          gmem1_rd_addr, gmem1_rd_len, gmem1_rd_avalid, gmem1_rd_dready, ...
          gmem0_wr_addr, gmem0_wr_len, gmem0_wr_valid, gmem0_wr_data, ...
          done, max_delta] = vi_sweep_soc_kernel( ...
          gmem2_rd_data, gmem2_rd_aready, gmem2_rd_dvalid, ...
          gmem1_rd_data, gmem1_rd_aready, gmem1_rd_dvalid, ...
          gmem0_wr_ready, gmem0_wr_bvalid, gmem0_wr_complete, ...
          start, map_x, map_y, cu_id)
%VI_SWEEP_SOC_KERNEL Cycle-stepped SoC kernel for the MATLAB VI algorithm.
%   The kernel loads the active map window from DDR into on-chip memories,
%   computes one CU sweep using the same strip partitioning as
%   vi_sweep_stream_algo, and writes the updated value table back to DDR.

    %#codegen

    p = vi_params();

    persistent state prev_start
    persistent old_value new_value penalty_table trans_words
    persistent active_map_x active_map_y active_cu
    persistent num_strips half_strips
    persistent load_word_idx total_value_words total_penalty_words
    persistent compute_si compute_y compute_x compute_t
    persistent write_word_idx global_max_delta

    if isempty(state)
        state = uint8(0);
        prev_start = false;
        old_value = zeros(p.SOC_MAP_Y_MAX, p.SOC_MAP_X_MAX, p.N_THETA, 'uint16');
        new_value = zeros(p.SOC_MAP_Y_MAX, p.SOC_MAP_X_MAX, p.N_THETA, 'uint16');
        penalty_table = zeros(p.SOC_MAP_Y_MAX, p.SOC_MAP_X_MAX, 'uint16');
        trans_words = zeros(1, p.TRANS_TABLE_SIZE, 'uint32');
        active_map_x = uint32(0);
        active_map_y = uint32(0);
        active_cu = uint32(0);
        num_strips = uint32(0);
        half_strips = uint32(0);
        load_word_idx = uint32(0);
        total_value_words = uint32(0);
        total_penalty_words = uint32(0);
        compute_si = uint32(0);
        compute_y = uint32(0);
        compute_x = uint32(0);
        compute_t = uint32(0);
        write_word_idx = uint32(0);
        global_max_delta = uint16(0);
    end

    gmem2_rd_addr = uint32(0);
    gmem2_rd_len = uint32(0);
    gmem2_rd_avalid = false;
    gmem2_rd_dready = true;

    gmem1_rd_addr = uint32(0);
    gmem1_rd_len = uint32(0);
    gmem1_rd_avalid = false;
    gmem1_rd_dready = true;

    gmem0_wr_addr = uint32(0);
    gmem0_wr_len = uint32(0);
    gmem0_wr_valid = false;
    gmem0_wr_data = uint32(0);

    done = false;
    max_delta = global_max_delta;

    start_edge = start && ~prev_start;
    prev_start = start;

    if state == uint8(0)
        if start_edge
            if map_x == 0 || map_y == 0 || ...
                    map_x > uint32(p.SOC_MAP_X_MAX) || ...
                    map_y > uint32(p.SOC_MAP_Y_MAX)
                done = true;
                max_delta = uint16(0);
                return;
            end

            active_map_x = uint32(map_x);
            active_map_y = uint32(map_y);
            active_cu = uint32(bitand(uint32(cu_id), uint32(1)));
            total_penalty_words = active_map_x * active_map_y;
            total_value_words = (active_map_x * active_map_y * uint32(p.N_THETA)) / uint32(2);
            num_strips = ceil_div(active_map_x, uint32(p.STRIP_W_MAX));
            half_strips = ceil_div(num_strips, uint32(2));
            load_word_idx = uint32(0);
            compute_si = uint32(0);
            compute_y = uint32(0);
            compute_x = uint32(0);
            compute_t = uint32(0);
            write_word_idx = uint32(0);
            global_max_delta = uint16(0);
            state = uint8(10);
        end
        return;
    end

    switch state
        case uint8(10) % issue trans-table read
            gmem1_rd_addr = uint32(p.SOC_TRANS_OFFSET_WORD * 4);
            gmem1_rd_len = uint32(p.TRANS_TABLE_SIZE);
            if gmem1_rd_aready
                gmem1_rd_avalid = true;
                load_word_idx = uint32(0);
                state = uint8(11);
            end

        case uint8(11) % receive trans-table
            gmem1_rd_len = uint32(p.TRANS_TABLE_SIZE);
            if gmem1_rd_dvalid
                trans_words(load_word_idx + 1) = uint32(gmem1_rd_data);
                if load_word_idx + 1 >= uint32(p.TRANS_TABLE_SIZE)
                    load_word_idx = uint32(0);
                    state = uint8(20);
                else
                    load_word_idx = load_word_idx + 1;
                end
            end

        case uint8(20) % issue value-table read
            gmem2_rd_addr = uint32(0);
            gmem2_rd_len = total_value_words;
            if gmem2_rd_aready
                gmem2_rd_avalid = true;
                load_word_idx = uint32(0);
                state = uint8(21);
            end

        case uint8(21) % receive value-table
            gmem2_rd_len = total_value_words;
            if gmem2_rd_dvalid
                [y0, x0, t0] = value_coords_from_linear(load_word_idx * uint32(2), active_map_x, p);
                old_value(y0, x0, t0) = uint16(bitand(uint32(gmem2_rd_data), uint32(65535)));
                new_value(y0, x0, t0) = old_value(y0, x0, t0);

                [y1, x1, t1, valid1] = value_coords_from_linear_opt(load_word_idx * uint32(2) + uint32(1), active_map_x, active_map_y, p);
                if valid1
                    old_value(y1, x1, t1) = uint16(bitshift(uint32(gmem2_rd_data), -16));
                    new_value(y1, x1, t1) = old_value(y1, x1, t1);
                end

                if load_word_idx + 1 >= total_value_words
                    load_word_idx = uint32(0);
                    state = uint8(30);
                else
                    load_word_idx = load_word_idx + 1;
                end
            end

        case uint8(30) % issue penalty-table read
            gmem1_rd_addr = uint32(0);
            gmem1_rd_len = total_penalty_words;
            if gmem1_rd_aready
                gmem1_rd_avalid = true;
                load_word_idx = uint32(0);
                state = uint8(31);
            end

        case uint8(31) % receive penalty-table
            gmem1_rd_len = total_penalty_words;
            if gmem1_rd_dvalid
                [py, px] = penalty_coords_from_linear(load_word_idx, active_map_x);
                penalty_val = uint16(bitand(uint32(gmem1_rd_data), uint32(65535)));
                penalty_table(py, px) = penalty_val;
                if penalty_val == p.PENALTY_GOAL
                    for gt = 1:p.N_THETA
                        old_value(py, px, gt) = uint16(0);
                        new_value(py, px, gt) = uint16(0);
                    end
                end

                if load_word_idx + 1 >= total_penalty_words
                    compute_si = uint32(0);
                    compute_y = uint32(0);
                    compute_x = uint32(0);
                    compute_t = uint32(0);
                    state = uint8(40);
                else
                    load_word_idx = load_word_idx + 1;
                end
            end

        case uint8(40) % compute one state per cycle
            if compute_si >= half_strips
                write_word_idx = uint32(0);
                state = uint8(50);
            else
                sx = selected_strip(compute_si, num_strips, active_cu);
                strip_x0 = sx * uint32(p.STRIP_W_MAX);
                strip_w = min_u32(uint32(p.STRIP_W_MAX), active_map_x - strip_x0);
                gx = strip_x0 + compute_x;
                gy = compute_y;
                theta = compute_t;

                [best_val, delta_val] = compute_state_value( ...
                    gx, gy, theta, active_map_x, active_map_y, active_cu, ...
                    old_value, penalty_table, trans_words, p);
                new_value(gy + 1, gx + 1, theta + 1) = best_val;
                if delta_val > global_max_delta
                    global_max_delta = delta_val;
                end
                max_delta = global_max_delta;

                if compute_t + 1 < uint32(p.N_THETA)
                    compute_t = compute_t + 1;
                else
                    compute_t = uint32(0);
                    if compute_x + 1 < strip_w
                        compute_x = compute_x + 1;
                    else
                        compute_x = uint32(0);
                        if compute_y + 1 < active_map_y
                            compute_y = compute_y + 1;
                        else
                            compute_y = uint32(0);
                            compute_si = compute_si + 1;
                        end
                    end
                end
            end

        case uint8(50) % stream writeback
            gmem0_wr_addr = uint32(0);
            gmem0_wr_len = total_value_words;
            if write_word_idx < total_value_words
                gmem0_wr_valid = true;
                gmem0_wr_data = pack_value_word(write_word_idx, new_value, active_map_x, active_map_y, p);
                write_word_idx = write_word_idx + 1;
            else
                state = uint8(51);
            end

        case uint8(51) % wait for write completion
            gmem0_wr_addr = uint32(0);
            gmem0_wr_len = total_value_words;
            if gmem0_wr_complete || gmem0_wr_bvalid || gmem0_wr_ready
                state = uint8(60);
            end

        otherwise % done
            done = true;
            max_delta = global_max_delta;
            if ~start
                state = uint8(0);
            end
    end
end

function q = ceil_div(a, b)
    q = idivide(a + b - 1, b, 'floor');
end

function m = min_u32(a, b)
    if a < b
        m = a;
    else
        m = b;
    end
end

function sx = selected_strip(si, num_strips, cu_id)
    if cu_id == 0
        sx = si;
    else
        sx = num_strips - 1 - si;
    end
end

function [iy, ix, it] = value_coords_from_linear(idx, map_x, p)
    cell_idx = idivide(idx, uint32(p.N_THETA), 'floor');
    it = mod(idx, uint32(p.N_THETA)) + 1;
    ix = mod(cell_idx, map_x) + 1;
    iy = idivide(cell_idx, map_x, 'floor') + 1;
end

function [iy, ix, it, valid] = value_coords_from_linear_opt(idx, map_x, map_y, p)
    total = map_x * map_y * uint32(p.N_THETA);
    if idx < total
        [iy, ix, it] = value_coords_from_linear(idx, map_x, p);
        valid = true;
    else
        iy = uint32(1);
        ix = uint32(1);
        it = uint32(1);
        valid = false;
    end
end

function [iy, ix] = penalty_coords_from_linear(idx, map_x)
    ix = mod(idx, map_x) + 1;
    iy = idivide(idx, map_x, 'floor') + 1;
end

function word = pack_value_word(word_idx, table, map_x, map_y, p)
    idx0 = word_idx * uint32(2);
    [y0, x0, t0] = value_coords_from_linear(idx0, map_x, p);
    lo = uint32(table(y0, x0, t0));
    [y1, x1, t1, valid1] = value_coords_from_linear_opt(idx0 + uint32(1), map_x, map_y, p);
    if valid1
        hi = bitshift(uint32(table(y1, x1, t1)), 16);
    else
        hi = uint32(0);
    end
    word = bitor(lo, hi);
end

function [best_val, delta_val] = compute_state_value(gx, gy, theta, map_x, map_y, cu_id, ...
                                                     old_value, penalty_table, trans_words, p)
    cell_pen = penalty_table(gy + 1, gx + 1);
    old_val = old_value(gy + 1, gx + 1, theta + 1);

    if cell_pen == p.PENALTY_GOAL
        best_val = uint16(0);
        delta_val = abs_u16(best_val, old_val);
        return;
    end

    if cell_pen == p.PENALTY_OBSTACLE
        best_val = old_val;
        delta_val = uint16(0);
        return;
    end

    best_cost = uint16(p.MAX_VALUE);

    for a = 1:p.N_ACTIONS
        base = (uint32(a - 1) * uint32(p.N_THETA) + theta) * ...
            uint32(p.TRANS_WORD_STRIDE) + uint32(1);
        n_outcomes = uint32(trans_words(base));
        accum = uint64(0);
        invalid = false;

        for k = 1:10
            if uint32(k) <= n_outcomes && ~invalid
                delta_word = trans_words(base + uint32(2 * k) - uint32(1));
                prob_word = trans_words(base + uint32(2 * k));

                dix = decode_int8_from_word(delta_word, 0);
                diy = decode_int8_from_word(delta_word, 8);
                dit = decode_int8_from_word(delta_word, 16);

                nx = int32(gx) + dix;
                if cu_id == 0
                    ny = int32(gy) + diy;
                else
                    ny = int32(gy) - diy;
                end

                if nx < 0 || nx >= int32(map_x) || ny < 0 || ny >= int32(map_y)
                    invalid = true;
                else
                    nt = int32(theta) + dit;
                    if nt < 0
                        nt = nt + int32(p.N_THETA);
                    elseif nt >= int32(p.N_THETA)
                        nt = nt - int32(p.N_THETA);
                    end

                    cost_val = cost_of_neighbor( ...
                        old_value(ny + 1, nx + 1, nt + 1), ...
                        penalty_table(ny + 1, nx + 1), p);
                    if cost_val == p.MAX_VALUE
                        invalid = true;
                    else
                        accum = accum + uint64(cost_val) * uint64(prob_word);
                    end
                end
            end
        end

        if invalid
            act_cost = uint16(p.MAX_VALUE);
        else
            raw_cost = uint64(idivide(accum, uint64(p.PROB_BASE), 'floor'));
            if raw_cost >= uint64(p.MAX_VALUE)
                act_cost = uint16(p.MAX_VALUE - 1);
            else
                act_cost = uint16(raw_cost);
            end
        end

        if act_cost < best_cost
            best_cost = act_cost;
        end
    end

    best_val = best_cost;
    delta_val = abs_u16(best_val, old_val);
end

function c = cost_of_neighbor(nv, np_raw, p)
    if nv == p.MAX_VALUE || np_raw == p.PENALTY_OBSTACLE
        c = p.MAX_VALUE;
        return;
    end

    if np_raw == p.PENALTY_GOAL
        np = uint32(0);
    else
        np = uint32(np_raw);
    end

    sum_val = uint32(nv) + np + uint32(1);
    if sum_val >= uint32(p.MAX_VALUE)
        c = uint16(p.MAX_VALUE - 1);
    else
        c = uint16(sum_val);
    end
end

function d = abs_u16(a, b)
    if a >= b
        d = a - b;
    else
        d = b - a;
    end
end

function v = decode_int8_from_word(word, shift_amt)
    raw = int32(bitand(bitshift(word, -shift_amt), uint32(255)));
    if raw >= 128
        v = raw - 256;
    else
        v = raw;
    end
end
