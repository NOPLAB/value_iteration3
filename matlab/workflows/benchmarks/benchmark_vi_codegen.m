function rows = benchmark_vi_codegen(varargin)
%BENCHMARK_VI_CODEGEN MATLAB vs MATLAB-Coder C MEX speed comparison.
%
%   For each bench_cases() entry, runs every implementation registered in
%   bench_impls() twice — once interpreted in MATLAB and once via the MATLAB
%   Coder generated MEX — then reports:
%       * iters / total ms / ms-per-iter for each
%       * speedup (matlab_ms / codegen_ms)
%       * value mismatch %% (expected to be 0; codegen is bit-exact)
%       * max absolute value error
%
%   Options (name/value):
%       'rebuild'    (false)   force codegen rebuild before benchmarking
%       'impls'      ({})      restrict to a subset of impl names
%       'cases'      ({})      restrict to a subset of bench case names
%       'max_sweeps' (200)     max sweeps for reference / fpga_mimic
%       'max_iters'  (1000)    max iters for frontier variants
%
%   Outputs:
%       rows                struct array of all (case, impl) results
%       Markdown tables to stdout, one per implementation
%       CSV at artifacts/benchmarks/results/benchmark_codegen_<timestamp>.csv

    opts.rebuild    = false;
    opts.impls      = {};
    opts.cases      = {};
    opts.max_sweeps = 200;
    opts.max_iters  = 1000;
    if mod(numel(varargin), 2) ~= 0
        error('benchmark_vi_codegen:badopt', ...
            'arguments must be name/value pairs');
    end
    for i = 1:2:numel(varargin)
        key = char(varargin{i});
        switch key
            case {'rebuild', 'impls', 'cases', 'max_sweeps', 'max_iters'}
                opts.(key) = varargin{i + 1};
            otherwise
                error('benchmark_vi_codegen:badopt', 'unknown option: %s', key);
        end
    end
    if islogical(opts.rebuild) || isnumeric(opts.rebuild)
        opts.rebuild = logical(opts.rebuild);
    end

    layout = vi_matlab_layout();
    setup_matlab_paths('src', 'tests', 'bench');

    impls = bench_impls(opts.max_sweeps, opts.max_iters);
    if ~isempty(opts.impls)
        keep = ismember({impls.name}, opts.impls);
        if ~any(keep)
            error('benchmark_vi_codegen:badimpl', ...
                'none of the requested impls match: %s', strjoin(opts.impls, ', '));
        end
        impls = impls(keep);
    end

    % Build (or reuse) the codegen MEX files. Restrict to the entries the
    % selected impls actually need so we don't pay for unrelated builds.
    entry_names = unique({impls.entry});
    fprintf('=== codegen build phase (%d entries) ===\n', numel(entry_names));
    codegen_build('entries', entry_names, 'rebuild', opts.rebuild);

    p = vi_params();
    transitions = gen_transitions('paper_mc');

    cases = bench_cases();
    if ~isempty(opts.cases)
        keep = ismember({cases.name}, opts.cases);
        if ~any(keep)
            error('benchmark_vi_codegen:badcase', ...
                'none of the requested cases match: %s', strjoin(opts.cases, ', '));
        end
        cases = cases(keep);
    end
    n_cases = numel(cases);
    n_impls = numel(impls);
    rows = repmat(empty_row(), 1, n_cases * n_impls);

    fprintf('\n=== benchmark phase (%d cases x %d impls) ===\n', n_cases, n_impls);
    idx = 1;
    for k = 1:n_cases
        c = cases(k);
        fprintf('  [%2d/%2d] %-14s\n', k, n_cases, c.name);

        [value0, penalty, ~, ~, goal_mask] = gen_test_map( ...
            c.map_x, c.map_y, c.type, c.opts);
        [v_oracle, ~, ~, ~] = vi_full_reference(value0, penalty, ...
            goal_mask, transitions, c.map_x, c.map_y, 0, opts.max_sweeps);

        % --- Warm up MATLAB JIT and codegen MEX caches. -------------------
        for j = 1:n_impls
            impls(j).matlab_fn(value0, penalty, goal_mask, transitions, ...
                c.map_x, c.map_y, 1);
            impls(j).codegen_fn(value0, penalty, goal_mask, transitions, ...
                c.map_x, c.map_y, 1);
        end

        for j = 1:n_impls
            im = impls(j);

            t0 = tic;
            [v_m, sw_m, mt_m] = im.matlab_fn(value0, penalty, goal_mask, ...
                transitions, c.map_x, c.map_y, im.cap);
            t_m = toc(t0) * 1000;

            t0 = tic;
            [v_c, sw_c, mt_c] = im.codegen_fn(value0, penalty, goal_mask, ...
                transitions, c.map_x, c.map_y, im.cap);
            t_c = toc(t0) * 1000;

            metrics = compute_value_metrics(v_m, v_c, goal_mask, penalty, p);
            oracle_metrics = compute_value_metrics(v_oracle, v_c, ...
                goal_mask, penalty, p);
            rows(idx) = build_row(im.name, c, sw_m, t_m, mt_m, ...
                sw_c, t_c, mt_c, metrics, oracle_metrics);
            idx = idx + 1;

            fprintf('    %-15s M=%7.1fms (%4d %s)  C=%7.1fms (%4d %s)  x%6.1f%s\n', ...
                im.name, t_m, sw_m, im.iter_label, ...
                t_c, sw_c, im.iter_label, ...
                t_m / max(t_c, eps), ...
                accuracy_flag(metrics));
        end
    end

    for j = 1:n_impls
        print_markdown(rows, impls(j).name, impls(j).title, impls(j).iter_label);
    end

    results_dir = layout.artifacts_benchmarks_results;
    if ~exist(results_dir, 'dir'); mkdir(results_dir); end
    csv_path = fullfile(results_dir, ...
        sprintf('benchmark_codegen_%s.csv', datestr(now, 'yyyymmdd_HHMMSS'))); %#ok<TNOW1,DATST>
    write_csv(rows, csv_path);
    fprintf('\nCSV written to %s\n', csv_path);
end

% ---------------------------------------------------------------------------
% Implementation registry
% ---------------------------------------------------------------------------

function impls = bench_impls(max_sweeps, max_iters)
%BENCH_IMPLS Registry of (impl name, MATLAB fn, MEX fn, iter cap, labels).
%   Each matlab_fn / codegen_fn shares the signature
%       [v_out, iters, metric] = fn(v, pen, goal, trans, map_x, map_y, cap)
%   Returning a uniform 3-tuple lets the bench loop stay generic.

    impls = struct( ...
        'name',       {'reference',  'fpga_mimic',  'frontier_2d', 'frontier_3d', 'frontier_stack', ...
                       'frontier_3d_tau', 'frontier_3d_topk', 'frontier_3d_coarse_theta', ...
                       'pyramid_sweep'}, ...
        'title',      {'Paper reference (vi_full_reference)', ...
                       'FPGA-mimic streaming (vi_sweep_stream_algo)', ...
                       'Frontier 2D (vi_frontier_2d)', ...
                       'Frontier 3D (vi_frontier_3d)', ...
                       'Frontier stack (vi_frontier_stack)', ...
                       'Frontier 3D tau=4 (vi_frontier_3d_tau)', ...
                       'Frontier 3D top-k=3 (vi_frontier_3d_topk)', ...
                       'Frontier 3D coarse theta/refine (vi_frontier_3d_coarse_theta)', ...
                       'Pyramid 2x2 sweep (vi_pyramid_sweep)'}, ...
        'entry',      {'vi_full_reference_entry', 'vi_sweep_stream_full_entry', ...
                       'vi_frontier_2d_entry', 'vi_frontier_3d_entry', ...
                       'vi_frontier_stack_entry', ...
                       'vi_frontier_3d_tau_entry', ...
                       'vi_frontier_3d_topk_entry', ...
                       'vi_frontier_3d_coarse_theta_entry', ...
                       'vi_pyramid_sweep_entry'}, ...
        'iter_label', {'sw', 'sw', 'it', 'it', 'it', 'it', 'it', 'it', 'sw'}, ...
        'cap',        {max_sweeps, max_sweeps, max_iters, max_iters, max_iters, ...
                       max_iters, max_iters, max_iters, max_sweeps}, ...
        'matlab_fn',  {@matlab_reference, @matlab_fpga_mimic, ...
                       @matlab_frontier_2d, @matlab_frontier_3d, @matlab_frontier_stack, ...
                       @matlab_frontier_3d_tau, @matlab_frontier_3d_topk, ...
                       @matlab_frontier_3d_coarse_theta, @matlab_pyramid_sweep}, ...
        'codegen_fn', {@codegen_reference, @codegen_fpga_mimic, ...
                       @codegen_frontier_2d, @codegen_frontier_3d, @codegen_frontier_stack, ...
                       @codegen_frontier_3d_tau, @codegen_frontier_3d_topk, ...
                       @codegen_frontier_3d_coarse_theta, @codegen_pyramid_sweep});
end

% --- MATLAB-side runners ---------------------------------------------------

function [v, sw, m] = matlab_reference(v0, pen, goal, trans, mx, my, cap)
    [v, ~, sw, m] = vi_full_reference(v0, pen, goal, trans, mx, my, 0, cap);
end

function [v, sw, m] = matlab_fpga_mimic(v0, pen, goal, trans, mx, my, cap)
    v = v0; sw = 0; m = inf;
    for s = 1:cap
        [v, d0] = vi_sweep_stream_algo(v, v, pen, goal, trans, mx, my, 0);
        [v, d1] = vi_sweep_stream_algo(v, v, pen, goal, trans, mx, my, 1);
        sw = s; m = max(d0, d1);
        if m == 0; break; end
    end
end

function [v, it, upd] = matlab_frontier_2d(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_2d(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = matlab_frontier_3d(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_3d(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = matlab_frontier_stack(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_stack(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = matlab_frontier_3d_tau(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_3d_tau(v0, pen, goal, trans, mx, my, cap, 4);
end

function [v, it, upd] = matlab_frontier_3d_topk(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_3d_topk(v0, pen, goal, trans, mx, my, cap, 3);
end

function [v, it, upd] = matlab_frontier_3d_coarse_theta(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_3d_coarse_theta(v0, pen, goal, trans, ...
        mx, my, cap, 3, floor(cap / 4));
end

function [v, sw, visited] = matlab_pyramid_sweep(v0, pen, goal, trans, mx, my, cap)
    [v, sw, visited] = vi_pyramid_sweep(v0, pen, goal, trans, ...
        mx, my, 0, cap, 4, 8, cap, 1);
end

% --- codegen-MEX runners ---------------------------------------------------

function [v, sw, m] = codegen_reference(v0, pen, goal, trans, mx, my, cap)
    [v, ~, sw, m] = vi_full_reference_entry_mex(v0, pen, goal, trans, mx, my, 0, cap);
end

function [v, sw, m] = codegen_fpga_mimic(v0, pen, goal, trans, mx, my, cap)
    [v, sw, m] = vi_sweep_stream_full_entry_mex(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = codegen_frontier_2d(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_2d_entry_mex(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = codegen_frontier_3d(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_3d_entry_mex(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = codegen_frontier_stack(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_stack_entry_mex(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = codegen_frontier_3d_tau(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_3d_tau_entry_mex(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = codegen_frontier_3d_topk(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_3d_topk_entry_mex(v0, pen, goal, trans, mx, my, cap);
end

function [v, it, upd] = codegen_frontier_3d_coarse_theta(v0, pen, goal, trans, mx, my, cap)
    [v, it, upd] = vi_frontier_3d_coarse_theta_entry_mex(v0, pen, goal, trans, mx, my, cap);
end

function [v, sw, visited] = codegen_pyramid_sweep(v0, pen, goal, trans, mx, my, cap)
    [v, sw, visited] = vi_pyramid_sweep_entry_mex(v0, pen, goal, trans, mx, my, cap);
end

% ---------------------------------------------------------------------------
% Row construction, metrics, reporting
% ---------------------------------------------------------------------------

function row = empty_row()
    row = struct( ...
        'impl', '', 'name', '', 'map_x', 0, 'map_y', 0, 'type', '', ...
        'matlab_iters', 0, 'matlab_total_ms', 0, 'matlab_per_iter_ms', 0, ...
        'matlab_metric', 0, ...
        'codegen_iters', 0, 'codegen_total_ms', 0, 'codegen_per_iter_ms', 0, ...
        'codegen_metric', 0, ...
        'speedup', 0, ...
        'value_mismatch_pct', 0, 'mean_abs_value_pct', 0, 'max_abs_value', 0, ...
        'oracle_value_mismatch_pct', 0, ...
        'oracle_mean_abs_value_pct', 0, ...
        'oracle_max_abs_value', 0);
end

function row = build_row(impl, c, sw_m, t_m, mt_m, sw_c, t_c, mt_c, ...
    metrics, oracle_metrics)
    row = struct( ...
        'impl',                impl, ...
        'name',                c.name, ...
        'map_x',               c.map_x, ...
        'map_y',               c.map_y, ...
        'type',                c.type, ...
        'matlab_iters',        sw_m, ...
        'matlab_total_ms',     t_m, ...
        'matlab_per_iter_ms',  t_m / max(sw_m, 1), ...
        'matlab_metric',       double(mt_m), ...
        'codegen_iters',       sw_c, ...
        'codegen_total_ms',    t_c, ...
        'codegen_per_iter_ms', t_c / max(sw_c, 1), ...
        'codegen_metric',      double(mt_c), ...
        'speedup',             t_m / max(t_c, eps), ...
        'value_mismatch_pct',  metrics.value_mismatch_pct, ...
        'mean_abs_value_pct',  metrics.mean_abs_value_pct, ...
        'max_abs_value',       metrics.max_abs_value, ...
        'oracle_value_mismatch_pct', oracle_metrics.value_mismatch_pct, ...
        'oracle_mean_abs_value_pct', oracle_metrics.mean_abs_value_pct, ...
        'oracle_max_abs_value',      oracle_metrics.max_abs_value);
end

function flag = accuracy_flag(metrics)
    if metrics.value_mismatch_pct == 0
        flag = '';
    else
        flag = sprintf('  ! vmiss=%.2f%%', metrics.value_mismatch_pct);
    end
end

function m = compute_value_metrics(v_a, v_b, goal_mask, penalty, p)
% Compares two value tables over the free (non-obstacle, non-goal) cells.
% v_a is treated as the reference for the percentage denominator.
    OB = double(p.PENALTY_OBSTACLE);
    free_mask = ~goal_mask & repmat(penalty ~= OB, [1, 1, p.N_THETA]);
    total = nnz(free_mask);

    if total == 0
        m = struct('value_mismatch_pct', 0, 'mean_abs_value_pct', 0, ...
            'max_abs_value', 0);
        return;
    end

    a = double(v_a(free_mask));
    b = double(v_b(free_mask));
    denom = max(a, 1);
    m = struct( ...
        'value_mismatch_pct', 100 * nnz(a ~= b) / total, ...
        'mean_abs_value_pct', 100 * mean(abs(b - a) ./ denom), ...
        'max_abs_value',      max(abs(b - a)));
end

function print_markdown(rows, impl, title, iter_label)
    sel = rows(strcmp({rows.impl}, impl));
    if isempty(sel); return; end
    fprintf('\n### %s\n\n', title);
    fprintf('| name | size | %s | matlab ms | matlab ms/%s | codegen ms | codegen ms/%s | speedup | vmiss%% | maxabs | oracle miss%% | oracle maxabs |\n', ...
        iter_label, iter_label, iter_label);
    fprintf('|------|------|----:|----------:|--------------:|-----------:|---------------:|--------:|-------:|-------:|-------------:|--------------:|\n');
    for k = 1:numel(sel)
        r = sel(k);
        fprintf('| %s | %dx%d | %d | %.1f | %.2f | %.1f | %.2f | %.2fx | %.2f | %.0f | %.2f | %.0f |\n', ...
            r.name, r.map_x, r.map_y, r.matlab_iters, ...
            r.matlab_total_ms,  r.matlab_per_iter_ms, ...
            r.codegen_total_ms, r.codegen_per_iter_ms, ...
            r.speedup, r.value_mismatch_pct, r.max_abs_value, ...
            r.oracle_value_mismatch_pct, r.oracle_max_abs_value);
    end
end

function write_csv(rows, path)
    fid = fopen(path, 'w');
    if fid < 0
        error('benchmark_vi_codegen:csv', 'cannot open %s for writing', path);
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, ['impl,name,map_x,map_y,type,' ...
        'matlab_iters,matlab_total_ms,matlab_per_iter_ms,matlab_metric,' ...
        'codegen_iters,codegen_total_ms,codegen_per_iter_ms,codegen_metric,' ...
        'speedup,value_mismatch_pct,mean_abs_value_pct,max_abs_value,' ...
        'oracle_value_mismatch_pct,oracle_mean_abs_value_pct,oracle_max_abs_value\n']);

    for k = 1:numel(rows)
        r = rows(k);
        fprintf(fid, '%s,%s,%d,%d,%s,%d,%.6f,%.6f,%.0f,%d,%.6f,%.6f,%.0f,%.6f,%.6f,%.6f,%.0f,%.6f,%.6f,%.0f\n', ...
            r.impl, r.name, r.map_x, r.map_y, r.type, ...
            r.matlab_iters, r.matlab_total_ms, r.matlab_per_iter_ms, r.matlab_metric, ...
            r.codegen_iters, r.codegen_total_ms, r.codegen_per_iter_ms, r.codegen_metric, ...
            r.speedup, r.value_mismatch_pct, r.mean_abs_value_pct, r.max_abs_value, ...
            r.oracle_value_mismatch_pct, r.oracle_mean_abs_value_pct, r.oracle_max_abs_value);
    end
end
