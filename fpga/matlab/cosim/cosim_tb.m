function cosim_tb()
%COSIM_TB HDL Verifier cosimulation testbench.
%   Runs generated HDL through Xsim and compares against MATLAB golden output.
%   Prerequisites:
%     1. Phase A (float) tests pass (tb_full_sweep)
%     2. Fixed-point conversion applied
%     3. HDL generated via hdlcoder.WorkflowAdvisor

    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'testbench'));

    cfg = cosim_config();

    % Ensure output directory exists
    if ~exist(cfg.output_dir, 'dir')
        mkdir(cfg.output_dir);
    end

    fprintf('=== HDL Cosimulation Testbench ===\n');
    fprintf('Simulator: %s\n', cfg.simulator);
    fprintf('HDL Language: %s\n', cfg.hdl_lang);

    for tc = 1:numel(cfg.tests)
        t = cfg.tests{tc};
        fprintf('\n--- Test: %s (%dx%d, %s) ---\n', t.name, t.mx, t.my, t.type);

        % Generate inputs
        [value, penalty, ~, ~] = gen_test_map(t.mx, t.my, t.type);
        trans = gen_transitions('trivial');

        % Run MATLAB golden model
        ml_value = value;
        for sweep = 1:t.sweeps
            [ml_value, d0] = vi_sweep_stream_algo(ml_value, ml_value, ...
                                                   penalty, trans, ...
                                                   t.mx, t.my, 0);
            [ml_value, d1] = vi_sweep_stream_algo(ml_value, ml_value, ...
                                                   penalty, trans, ...
                                                   t.mx, t.my, 1);
            if max(d0, d1) == 0, break; end
        end

        % TODO: After HDL is generated, add Xsim cosimulation commands here:
        % 1. filtertbench = hdlverifier.FILSimulation(...)
        % 2. filtertbench.InputSignals = {value_flat, penalty_flat, trans};
        % 3. filtertbench.run()
        % 4. Compare filtertbench.OutputSignals against ml_value
        %
        % For now, save golden data for manual cosim verification.
        save(fullfile(cfg.output_dir, [t.name '_golden.mat']), ...
             'value', 'penalty', 'trans', 'ml_value', 't');
        fprintf('  Golden data saved to %s_golden.mat\n', t.name);
    end

    fprintf('\n=== Cosimulation setup complete ===\n');
    fprintf('Next steps:\n');
    fprintf('  1. Generate HDL from Simulink: hdlcoder.WorkflowAdvisor\n');
    fprintf('  2. Update cosim_tb.m with hdlverifier.FILSimulation calls\n');
    fprintf('  3. Re-run cosim_tb to compare HDL output vs golden\n');
end
