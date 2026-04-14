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
    cleanup = onCleanup(@() close_system_if_loaded(model_name));

    [dut_path, workflow_cfg, chosen_target, chosen_reference, preflight] = ...
        configure_workflow(model_name, cfg, build_dir);
    if isempty(preflight)
        fprintf('\nConfigured target platform: %s\n', chosen_target);
        fprintf('Configured reference design: %s\n', chosen_reference);
        fprintf('Project folder: %s\n', workflow_cfg.ProjectFolder);
    end
    preflight = [preflight, validate_batch_prereqs()];
    if ~isempty(preflight)
        error('build_bitstream:PreflightFailed\n%s', strjoin(preflight, newline));
    end

    if cfg.run_model_analyzer
        fprintf('\n--- Preflight: SoC model analysis ---\n');
        socModelAnalyzer(model_name);
    end

    fprintf('\n--- Step 1: IP Core Generation ---\n');
    fprintf('--- Step 2: Build Bitstream ---\n');
    hdlcoder.runWorkflow(dut_path, workflow_cfg, 'Verbosity', 'on');

    artifacts = collect_artifacts(workflow_cfg.ProjectFolder);

    fprintf('\n=== Bitstream generation complete ===\n');
    print_artifact_group('Bitstream', artifacts.bit);
    print_artifact_group('Hardware handoff', artifacts.hwh);
    print_artifact_group('XSA', artifacts.xsa);
    print_artifact_group('Logs', artifacts.log);
    clear cleanup
    close_system_if_loaded(model_name);
end

function [dut_path, hWC, chosen_target, chosen_reference, issues] = ...
        configure_workflow(model_name, cfg, build_dir)
    issues = {};
    dut_path = resolve_dut_path(model_name);

    hdlset_param(model_name, 'Workflow', cfg.workflow);
    hdlset_param(model_name, 'SynthesisTool', 'Xilinx Vivado');

    if ~isempty(cfg.reference_design_path)
        hdlset_param(model_name, 'ReferenceDesignPath', cfg.reference_design_path);
    end

    [chosen_target, target_issue] = choose_target_platform(model_name, cfg);
    if ~isempty(target_issue)
        issues{end + 1} = target_issue; %#ok<AGROW>
    end

    chosen_reference = '';
    if isempty(issues)
        [chosen_reference, reference_issue] = choose_reference_design( ...
            model_name, cfg, chosen_target);
        if ~isempty(reference_issue)
            issues{end + 1} = reference_issue; %#ok<AGROW>
        end
    end

    hWC = create_workflow_config(cfg, build_dir);
end

function hWC = create_workflow_config(cfg, build_dir)
    hWC = hdlcoder.WorkflowConfig( ...
        'SynthesisTool', 'Xilinx Vivado', ...
        'TargetWorkflow', cfg.workflow);

    hWC.ProjectFolder = fullfile(build_dir, cfg.project_dirname);
    hWC.AllowUnsupportedToolVersion = cfg.allow_unsupported_tool_version;
    hWC.IgnoreToolVersionMismatch = cfg.ignore_tool_version_mismatch;
    if ~isempty(cfg.reference_design_tool_version)
        hWC.ReferenceDesignToolVersion = cfg.reference_design_tool_version;
    end

    hWC.RunTaskGenerateRTLCodeAndIPCore = true;
    hWC.RunTaskCreateProject = true;
    hWC.RunTaskGenerateSoftwareInterface = cfg.generate_software_interface;
    hWC.RunTaskBuildFPGABitstream = true;
    hWC.RunTaskProgramTargetDevice = false;

    hWC.GenerateIPCoreReport = true;
    hWC.RunExternalBuild = cfg.run_external_build;
    hWC.MaxNumOfCoresForBuild = cfg.max_num_cores_for_build;
    hWC.GenerateSoftwareInterfaceModel = cfg.generate_software_interface_model;
    hWC.GenerateHostInterfaceScript = cfg.generate_host_interface_script;
    hWC.GenerateHostInterfaceModel = false;
end

function [chosen_target, issue] = choose_target_platform(model_name, cfg)
    issue = '';
    candidates = cfg.target_platform_candidates;
    if cfg.allow_target_platform_fallback
        candidates = [candidates, cfg.fallback_target_platform_candidates];
    end

    [chosen_target, last_error] = choose_hdl_param(model_name, 'TargetPlatform', candidates);
    if isempty(chosen_target)
        detail = 'Install the Ultra96-V2 HDL Coder BSP or update soc_config.m.';
        if ~cfg.allow_target_platform_fallback && ~isempty(cfg.fallback_target_platform_candidates)
            detail = sprintf(['%s A known fallback is `%s`, but fallback is disabled ', ...
                              'to avoid building for the wrong board.'], ...
                             detail, cfg.fallback_target_platform_candidates{1});
        end
        issue = sprintf(['Unable to resolve HDL Coder target platform.\n', ...
                         '  Tried: %s\n', ...
                         '  Last error: %s\n', ...
                         '  Action: %s'], ...
                        strjoin(candidates, ', '), last_error, detail);
    end
end

function [chosen_reference, issue] = choose_reference_design(model_name, cfg, chosen_target)
    issue = '';
    candidates = reference_design_candidates(cfg, chosen_target);
    [chosen_reference, last_error] = choose_hdl_param(model_name, 'ReferenceDesign', candidates);
    if isempty(chosen_reference)
        issue = sprintf(['Unable to resolve reference design for target platform `%s`.\n', ...
                         '  Tried: %s\n', ...
                         '  Last error: %s'], ...
                        chosen_target, strjoin(candidates, ', '), last_error);
    end
end

function candidates = reference_design_candidates(cfg, chosen_target)
    candidates = cfg.reference_design_candidates;
    if contains(chosen_target, 'ZCU102')
        candidates = [ ...
            {'Default system with External DDR4 Memory Access'}, ...
            candidates ...
        ];
    elseif contains(chosen_target, 'ZC706')
        candidates = [ ...
            {'Default system with External DDR3 Memory Access'}, ...
            candidates ...
        ];
    end
    candidates = unique(candidates, 'stable');
end

function [chosen_value, last_error] = choose_hdl_param(model_name, param_name, candidates)
    chosen_value = '';
    last_error = 'No candidates provided.';
    for idx = 1:numel(candidates)
        candidate = candidates{idx};
        if isempty(candidate)
            continue;
        end
        try
            hdlset_param(model_name, param_name, candidate);
            chosen_value = char(string(hdlget_param(model_name, param_name)));
            return;
        catch ME
            last_error = sanitize_message(ME.message);
        end
    end
end

function dut_path = resolve_dut_path(model_name)
    dut_path = char(string(hdlget_param(model_name, 'HDLSubsystem')));
    if isempty(dut_path)
        dut_path = model_name;
    end
end

function issues = validate_batch_prereqs()
    issues = {};

    cpp_cfg = mex.getCompilerConfigurations('C++', 'Selected');
    if isempty(cpp_cfg)
        issues{end + 1} = ['No supported C++ compiler is configured for MATLAB code generation. ', ...
                           'Run `mex -setup C++` or install MinGW-w64 support, then restart MATLAB.']; %#ok<AGROW>
    else
        fprintf('C++ compiler: %s\n', cpp_cfg.Name);
    end

    [vivado_ok, vivado_msg] = validate_vivado();
    if vivado_ok
        fprintf('Vivado: %s\n', vivado_msg);
    else
        issues{end + 1} = vivado_msg; %#ok<AGROW>
    end
end

function [ok, message] = validate_vivado()
    if ispc
        [status, output] = system('where vivado');
    else
        [status, output] = system('which vivado');
    end
    if status ~= 0
        ok = false;
        message = ['Vivado is not on PATH. Add the Xilinx/Vivado bin directory ', ...
                   'before running `make matlab-bitstream`.'];
        return;
    end

    ok = true;
    lines = regexp(strtrim(output), '\r?\n', 'split');
    message = strtrim(lines{1});
end

function artifacts = collect_artifacts(project_folder)
    artifacts.bit = collect_files(project_folder, '*.bit');
    artifacts.hwh = collect_files(project_folder, '*.hwh');
    artifacts.xsa = collect_files(project_folder, '*.xsa');
    artifacts.log = [ ...
        collect_files(project_folder, '*.log'), ...
        collect_files(project_folder, '*.jou') ...
    ];
end

function files = collect_files(root_dir, pattern)
    if ~exist(root_dir, 'dir')
        files = {};
        return;
    end

    entries = dir(fullfile(root_dir, '**', pattern));
    files = cell(1, numel(entries));
    for idx = 1:numel(entries)
        files{idx} = fullfile(entries(idx).folder, entries(idx).name);
    end
    files = unique(files, 'stable');
end

function print_artifact_group(label, files)
    if isempty(files)
        fprintf('%s: none found\n', label);
        return;
    end

    fprintf('%s:\n', label);
    for idx = 1:numel(files)
        fprintf('  %s\n', files{idx});
    end
end

function message = sanitize_message(message)
    message = strrep(message, newline, ' ');
    message = regexprep(message, '\s+', ' ');
end

function close_system_if_loaded(model_name)
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
end
