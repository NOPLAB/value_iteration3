function cfg = cosim_config()
%COSIM_CONFIG HDL Verifier cosimulation configuration.
%   Returns a struct with cosimulation parameters.

    layout = vi_matlab_layout();

    cfg.simulator = 'Vivado Simulator';  % Xsim
    cfg.hdl_lang = 'Verilog';
    cfg.clock_period_ns = 10;  % 100 MHz target
    cfg.reset_cycles = 5;

    % Test configurations (same as tb_full_sweep)
    cfg.tests = {
        struct('name','small',   'mx',8,  'my',8,  'type','empty',    'sweeps',20)
        struct('name','medium',  'mx',32, 'my',32, 'type','empty',    'sweeps',50)
        struct('name','sentinel','mx',8,  'my',8,  'type','sentinel', 'sweeps',20)
    };

    % Output directory for waveforms and logs
    cfg.output_dir = layout.artifacts_cosim;
end
