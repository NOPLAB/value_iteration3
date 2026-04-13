function build_bitstream()
%BUILD_BITSTREAM Generate bitstream via SoC Builder workflow.
%   Prerequisites:
%     1. Simulink model configured with SoC Blockset
%     2. HDL generation verified via cosimulation
%     3. Vivado on PATH

    cfg = soc_config();
    model_name = 'vi_sweep_stream_matlab';
    model_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'model');
    build_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'build');

    fprintf('=== SoC Builder Bitstream Generation ===\n');
    fprintf('Board: %s\n', cfg.board);
    fprintf('Device: %s\n', cfg.device);
    fprintf('Clock: %d MHz\n', cfg.clock_freq_mhz);
    fprintf('Build dir: %s\n', build_dir);

    if ~exist(build_dir, 'dir')
        mkdir(build_dir);
    end

    % Load model
    addpath(model_dir);
    load_system(model_name);

    % Run SoC Builder workflow
    % Step 1: Generate IP Core
    fprintf('\n--- Step 1: IP Core Generation ---\n');
    % hdlcoder.WorkflowAdvisor(model_name) in interactive mode
    % For batch: use hdlworkflow object
    % hw = hdlcoder.WorkflowConfig('SynthesisTool', 'Xilinx Vivado', ...
    %     'TargetWorkflow', 'IP Core Generation');
    % hw.run();

    % Step 2: Build Bitstream
    fprintf('--- Step 2: Build Bitstream ---\n');
    % Automated via SoC Builder:
    % socModelAnalyzer(model_name);
    % socBuildModel(model_name, 'BuildAction', 'Build');

    fprintf('\n=== Bitstream generation workflow ready ===\n');
    fprintf('Run interactively:\n');
    fprintf('  1. Open model: open_system(''%s'')\n', model_name);
    fprintf('  2. Launch: HDL Workflow Advisor\n');
    fprintf('  3. Target: IP Core Generation for SoC Builder\n');
    fprintf('  4. Board: %s\n', cfg.board);
    fprintf('  5. Generate and build\n');
end
