function fp_config()
%FP_CONFIG Configure Fixed-Point Advisor for the VI streaming kernel.
%   Run this after Phase A (floating-point) verification passes.
%   Uses tb_full_sweep test data to analyze dynamic range.

    setup_matlab_paths('src', 'tests');
    p = vi_params();

    % --- Define fixed-point type proposals ---
    % These are the target types matching vi_stream_types.h.
    % Fixed-Point Advisor will verify they have sufficient range.

    T = struct();

    % value_t: uint16, no fractional bits
    T.value = numerictype('Signed', false, 'WordLength', 16, 'FractionLength', 0);

    % penalty_t: uint16, no fractional bits
    T.penalty = numerictype('Signed', false, 'WordLength', 16, 'FractionLength', 0);

    % offset_t: int8
    T.offset = numerictype('Signed', true, 'WordLength', 8, 'FractionLength', 0);

    % cost_of intermediate: uint17 for nv + np addition
    T.cost_sum = numerictype('Signed', false, 'WordLength', 17, 'FractionLength', 0);

    % Display
    fprintf('Fixed-point type proposals:\n');
    fn = fieldnames(T);
    for i = 1:numel(fn)
        ft = T.(fn{i});
        fprintf('  %-12s: %s, W=%d, F=%d\n', fn{i}, ...
            ternary(ft.Signed, 'signed', 'unsigned'), ...
            ft.WordLength, ft.FractionLength);
    end

    % --- Generate instrumented test data ---
    fprintf('\nGenerating test data for range analysis...\n');
    [value, penalty, ~, ~, goal_mask] = gen_test_map(32, 32, 'empty');
    trans = gen_transitions('trivial');

    % Run a few sweeps to collect representative data
    for sweep = 1:10
        [value, ~] = vi_sweep_stream_algo(value, value, penalty, goal_mask, trans, 32, 32, 0);
        [value, ~] = vi_sweep_stream_algo(value, value, penalty, goal_mask, trans, 32, 32, 1);
    end

    % Report range of converged values
    valid = value(value < double(p.MAX_VALUE));
    if ~isempty(valid)
        fprintf('Value range after convergence: [%g, %g]\n', min(valid), max(valid));
        fprintf('Requires %d bits (unsigned)\n', ceil(log2(max(valid)+1)));
    end

    fprintf('\nfp_config complete. Run Fixed-Point Advisor from Simulink to apply.\n');
end

function r = ternary(cond, a, b)
    if cond, r = a; else, r = b; end
end
