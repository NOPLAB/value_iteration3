function layout = vi_matlab_layout()
%VI_MATLAB_LAYOUT Canonical directory layout for the MATLAB subtree.

    root = fileparts(mfilename('fullpath'));

    layout = struct();
    layout.root = root;

    layout.src = fullfile(root, 'src');

    layout.workflows_benchmarks = fullfile(root, 'workflows', 'benchmarks');
    layout.workflows_validation_tests = fullfile(root, 'workflows', 'validation', 'tests');

    layout.platforms = fullfile(root, 'platforms');
    layout.platforms_fpga = fullfile(layout.platforms, 'fpga');
    layout.platforms_fpga_board_support = fullfile(layout.platforms_fpga, 'board_support');
    layout.platforms_fpga_export = fullfile(layout.platforms_fpga, 'export');
    layout.platforms_fpga_model = fullfile(layout.platforms_fpga, 'model');

    layout.artifacts = fullfile(root, 'artifacts');
    layout.artifacts_benchmarks = fullfile(layout.artifacts, 'benchmarks');
    layout.artifacts_benchmarks_results = fullfile(layout.artifacts_benchmarks, 'results');
    layout.artifacts_build = fullfile(layout.artifacts, 'build');
    layout.artifacts_build_repo_ip = fullfile(layout.artifacts_build, 'repo_ip_prj');
end
