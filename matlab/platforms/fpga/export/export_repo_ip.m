function export_repo_ip(varargin)
%EXPORT_REPO_IP Generate MATLAB HDL Coder IP for the repo Vivado flow.
%   EXPORT_REPO_IP() regenerates the Simulink model, runs HDL Coder IP core
%   generation, and exports the packaged IP into fpga/build/matlab_ip_repo.
%
%   EXPORT_REPO_IP(OUTPUT_REPO) overrides the destination repository path.

    layout = vi_matlab_layout();
    repo_root = fileparts(layout.root);
    model_dir = layout.platforms_fpga_model;
    build_root = layout.artifacts_build_repo_ip;
    default_output_repo = fullfile(repo_root, 'fpga', 'build', 'matlab_ip_repo');
    default_header = fullfile(repo_root, 'driver', 'uio', 'generated', ...
                              'xalgorithm_ip_addr.h');

    if nargin >= 1 && ~isempty(varargin{1})
        output_repo = char(string(varargin{1}));
    else
        output_repo = default_output_repo;
    end
    if nargin >= 2 && ~isempty(varargin{2})
        output_header = char(string(varargin{2}));
    else
        output_header = default_header;
    end

    fprintf('=== MATLAB HDL Coder IP export ===\n');
    fprintf('Repo root: %s\n', repo_root);
    fprintf('IP repo:   %s\n', output_repo);
    fprintf('Header:    %s\n', output_header);

    setup_matlab_paths('src', 'fpga-export');

    if bdIsLoaded('vi_sweep_stream_matlab')
        close_system('vi_sweep_stream_matlab', 0);
    end

    create_model();
    load_system('vi_sweep_stream_matlab');
    cleanup = onCleanup(@() close_system_if_loaded('vi_sweep_stream_matlab'));

    configure_ip_workflow_target('vi_sweep_stream_matlab');

    project_folder = prepare_clean_dir(build_root);

    hWC = hdlcoder.WorkflowConfig( ...
        'SynthesisTool', 'Xilinx Vivado', ...
        'TargetWorkflow', 'IP Core Generation');
    hWC.ProjectFolder = project_folder;
    hWC.AllowUnsupportedToolVersion = true;
    hWC.IgnoreToolVersionMismatch = true;
    hWC.RunTaskGenerateRTLCodeAndIPCore = true;
    hWC.RunTaskCreateProject = false;
    hWC.RunTaskBuildFPGABitstream = false;
    hWC.RunTaskProgramTargetDevice = false;
    hWC.GenerateIPCoreReport = true;

    dut_path = char(string(hdlget_param('vi_sweep_stream_matlab', 'HDLSubsystem')));
    if isempty(dut_path)
        dut_path = 'vi_sweep_stream_matlab/Algorithm';
    end

    workflow_error = [];
    try
        hdlcoder.runWorkflow(dut_path, hWC, 'Verbosity', 'on');
    catch ME
        workflow_error = ME;
    end

    ip_root = resolve_generated_ip_root(project_folder);
    ensure_ip_is_packaged(ip_root);
    if ~isempty(workflow_error)
        fprintf(['HDL Workflow reported an error, but packaged IP artifacts ', ...
                 'were found. Continuing with exported IP.\n']);
        fprintf('Workflow error: %s\n', ...
                sanitize_message(workflow_error.message));
    end
    output_repo = prepare_clean_dir(output_repo);
    copyfile(ip_root, fullfile(output_repo, 'Algorithm_ip_v1_0'));

    addr_header = fullfile(ip_root, 'include', 'Algorithm_ip_addr.h');
    if exist(addr_header, 'file')
        [header_dir, ~, ~] = fileparts(output_header);
        if ~exist(header_dir, 'dir')
            mkdir(header_dir);
        end
        copyfile(addr_header, output_header);
    end

    fprintf('Exported IP: %s\n', fullfile(output_repo, 'Algorithm_ip_v1_0'));
    if exist(output_header, 'file')
        fprintf('Exported header: %s\n', output_header);
    end

    clear cleanup
    close_system_if_loaded('vi_sweep_stream_matlab');
end

function configure_ip_workflow_target(model_name)
    hdlset_param(model_name, 'Workflow', 'IP Core Generation');
    hdlset_param(model_name, 'SynthesisTool', 'Xilinx Vivado');

    candidates = { ...
        'Avnet Ultra96-V2', ...
        'Xilinx Zynq UltraScale+ MPSoC ZCU102 Evaluation Kit' ...
    };

    last_error = 'No target platform candidates provided.';
    for idx = 1:numel(candidates)
        candidate = candidates{idx};
        try
            hdlset_param(model_name, 'TargetPlatform', candidate);
            fprintf('Target platform: %s\n', ...
                    char(string(hdlget_param(model_name, 'TargetPlatform'))));
            return;
        catch ME
            last_error = sanitize_message(ME.message);
        end
    end

    error('export_repo_ip:TargetPlatformNotFound', ...
          ['Unable to resolve an HDL Coder target platform for IP export. ', ...
           'Last error: %s'], ...
          last_error);
end

function target_dir = prepare_clean_dir(target_dir)
    if exist(target_dir, 'dir')
        ensure_safe_generated_path(target_dir);
        [ok, msg, msgid] = rmdir(target_dir, 's');
        if ~ok
            error('export_repo_ip:CleanupFailed', ...
                  'Failed to remove `%s` (%s: %s).', target_dir, msgid, msg);
        end
    end

    [ok, msg, msgid] = mkdir(target_dir);
    if ~ok
        error('export_repo_ip:MkdirFailed', ...
              'Failed to create `%s` (%s: %s).', target_dir, msgid, msg);
    end
end

function ensure_safe_generated_path(target_dir)
    target_norm = lower(char(string(target_dir)));
    layout = vi_matlab_layout();
    repo_norm = lower(char(string(layout.root)));

    allowed_roots = { ...
        lower(char(string(layout.artifacts_build))), ...
        fullfile(repo_norm, 'fpga', 'build') ...
    };

    for idx = 1:numel(allowed_roots)
        root = allowed_roots{idx};
        prefix = [root filesep];
        if strcmp(target_norm, root) || startsWith(target_norm, prefix)
            return;
        end
    end

    error('export_repo_ip:UnsafePath', ...
          'Refusing to delete path outside generated build roots: %s', ...
          target_dir);
end

function ip_root = resolve_generated_ip_root(project_folder)
    candidates = {fullfile(project_folder, 'ipcore', 'Algorithm_ip_v1_0')};

    for idx = 1:numel(candidates)
        candidate = candidates{idx};
        if exist(candidate, 'dir')
            ip_root = candidate;
            return;
        end
    end

    entries = dir(fullfile(project_folder, '**', 'vivado_ip_package.tcl'));
    for idx = 1:numel(entries)
        candidate = fileparts(entries(idx).folder);
        if strcmp(char(string(candidate)), ...
                  char(string(fullfile(project_folder, 'ipcore', ...
                                        'Algorithm_ip_v1_0'))))
            ip_root = candidate;
            return;
        end
    end

    error('export_repo_ip:IpNotFound', ...
          'Could not locate packaged IP under `%s`.', project_folder);
end

function ensure_ip_is_packaged(ip_root)
    component_file = fullfile(ip_root, 'component.xml');
    if exist(component_file, 'file')
        return;
    end

    package_tcl = fullfile(ip_root, 'prj_ip', 'vivado_ip_package.tcl');
    if ~exist(package_tcl, 'file')
        error('export_repo_ip:PackagerScriptMissing', ...
              'Vivado package script not found: %s', package_tcl);
    end

    fprintf(['Packaged IP metadata was not generated by HDL Coder. ', ...
             'Running Vivado IP packager...\n']);
    run_vivado_packager(package_tcl, fullfile(ip_root, 'prj_ip'));

    if ~exist(component_file, 'file')
        error('export_repo_ip:PackagerOutputMissing', ...
              'Vivado completed without producing `%s`.', component_file);
    end
end

function run_vivado_packager(package_tcl, working_dir)
    layout = vi_matlab_layout();
    repo_root = fileparts(layout.root);
    vivado_state_root = fullfile(repo_root, 'fpga', 'build', '.vivado_user_data');
    ensure_dir(vivado_state_root);
    ensure_dir(fullfile(vivado_state_root, 'AppData'));
    ensure_dir(fullfile(vivado_state_root, 'LocalAppData'));

    original_appdata = getenv('APPDATA');
    original_localappdata = getenv('LOCALAPPDATA');
    restore_env = onCleanup(@() restore_vivado_env( ...
        original_appdata, original_localappdata));

    setenv('APPDATA', fullfile(vivado_state_root, 'AppData'));
    setenv('LOCALAPPDATA', fullfile(vivado_state_root, 'LocalAppData'));

    command = sprintf(['cmd /c "cd /d ""%s"" && vivado -mode batch ', ...
                       '-source ""%s"""'], ...
                      working_dir, package_tcl);
    [status, output] = system(command);
    fprintf('%s', output);
    if status ~= 0
        error('export_repo_ip:VivadoPackagerFailed', ...
              'Vivado IP packager failed with exit code %d.', status);
    end

    clear restore_env
    restore_vivado_env(original_appdata, original_localappdata);
end

function ensure_dir(target_dir)
    if exist(target_dir, 'dir')
        return;
    end

    [ok, msg, msgid] = mkdir(target_dir);
    if ~ok
        error('export_repo_ip:MkdirFailed', ...
              'Failed to create `%s` (%s: %s).', target_dir, msgid, msg);
    end
end

function restore_vivado_env(original_appdata, original_localappdata)
    setenv('APPDATA', original_appdata);
    setenv('LOCALAPPDATA', original_localappdata);
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
