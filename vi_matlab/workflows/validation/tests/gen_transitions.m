function trans = gen_transitions(mode)
%GEN_TRANSITIONS Generate packed transition table as uint32 column vector.
%   mode:
%     'trivial'  - deterministic 1-step test transitions
%     'paper_mc' - Monte Carlo transitions from the paper/ROS implementation

    persistent cache

    if isempty(cache)
        cache = struct();
    end
    if isfield(cache, mode)
        trans = cache.(mode);
        return;
    end

    p = vi_params();

    switch mode
        case 'trivial'
            model.n_outcomes = zeros(p.N_ACTIONS, p.N_THETA);
            model.dix = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
            model.diy = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
            model.dit = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
            model.prob = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
            for it = 1:p.N_THETA
                model.n_outcomes(1, it) = 1;
                model.dix(1, it, 1) = 1;
                model.prob(1, it, 1) = p.PROB_BASE;

                model.n_outcomes(2, it) = 1;
                model.dix(2, it, 1) = -1;
                model.prob(2, it, 1) = p.PROB_BASE;

                for a = 3:p.N_ACTIONS
                    model.n_outcomes(a, it) = 1;
                    model.prob(a, it, 1) = p.PROB_BASE;
                end
            end

        case 'paper_mc'
            model = build_monte_carlo_model(p, 0.05);

        otherwise
            error('Unknown mode: %s', mode);
    end

    trans = pack_model(model, p);
    cache.(mode) = trans;
end

function model = build_monte_carlo_model(p, xy_resolution)
    t_resolution = 360 / p.N_THETA;
    xy_sample_num = 2 ^ p.RESOLUTION_XY_BIT;
    t_sample_num = 2 ^ p.RESOLUTION_T_BIT;
    xy_step = xy_resolution / xy_sample_num;
    t_step = t_resolution / t_sample_num;
    ox_vals = 0.5 * xy_step + (0:xy_sample_num-1) * xy_step;
    oy_vals = 0.5 * xy_step + (0:xy_sample_num-1) * xy_step;
    ot_vals = 0.5 * t_step + (0:t_sample_num-1) * t_step;
    [oy_grid, ox_grid, ot_grid] = ndgrid(oy_vals, ox_vals, ot_vals);

    model.n_outcomes = zeros(p.N_ACTIONS, p.N_THETA);
    model.dix = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
    model.diy = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
    model.dit = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
    model.prob = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);

    for a = 1:p.N_ACTIONS
        for it = 1:p.N_THETA
            theta_origin = (it - 1) * t_resolution;
            ang = (ot_grid + theta_origin) / 180 * pi;
            dx = ox_grid + p.ACTION_FW(a) * cos(ang);
            dy = oy_grid + p.ACTION_FW(a) * sin(ang);
            dt = ot_grid + theta_origin + p.ACTION_ROT(a);
            dt = mod(dt, 360);

            dix = floor(abs(dx) / xy_resolution);
            dix(dx < 0) = -dix(dx < 0) - 1;

            diy = floor(abs(dy) / xy_resolution);
            diy(dy < 0) = -diy(dy < 0) - 1;

            dit_abs = floor(dt / t_resolution);
            dit = dit_abs - (it - 1);
            dit(dit > p.N_THETA / 2) = dit(dit > p.N_THETA / 2) - p.N_THETA;
            dit(dit < -p.N_THETA / 2) = dit(dit < -p.N_THETA / 2) + p.N_THETA;
            rows = [dix(:), diy(:), dit(:)];
            [uniq_rows, ~, ic] = unique(rows, 'rows', 'stable');
            counts = accumarray(ic, 1);

            n_out = size(uniq_rows, 1);
            if n_out > p.MAX_OUTCOMES
                error('MAX_OUTCOMES too small: got %d, need > %d', n_out, p.MAX_OUTCOMES);
            end

            % Stable ordering keeps packed tables deterministic.
            parsed = sortrows([uniq_rows, counts], [1, 2, 3]);
            model.n_outcomes(a, it) = n_out;
            for k = 1:n_out
                model.dix(a, it, k) = parsed(k, 1);
                model.diy(a, it, k) = parsed(k, 2);
                model.dit(a, it, k) = parsed(k, 3);
                model.prob(a, it, k) = parsed(k, 4);
            end
        end
    end
end

function trans = pack_model(model, p)
    trans = zeros(p.TRANS_TABLE_SIZE, 1, 'uint32');

    for a = 1:p.N_ACTIONS
        for it = 1:p.N_THETA
            base = ((a - 1) * p.N_THETA + (it - 1)) * p.TRANS_WORD_STRIDE + 1;
            trans(base) = uint32(model.n_outcomes(a, it));
            for k = 1:model.n_outcomes(a, it)
                word0 = pack_delta(model.dix(a, it, k), ...
                                   model.diy(a, it, k), ...
                                   model.dit(a, it, k));
                word1 = uint32(model.prob(a, it, k));
                trans(base + 2 * k - 1) = word0;
                trans(base + 2 * k) = word1;
            end
        end
    end
end

function w = pack_delta(dix, diy, dit)
    b0 = typecast(int8(dix), 'uint8');
    b1 = typecast(int8(diy), 'uint8');
    b2 = typecast(int8(dit), 'uint8');
    w = uint32(b0) + bitshift(uint32(b1), 8) + bitshift(uint32(b2), 16);
end
