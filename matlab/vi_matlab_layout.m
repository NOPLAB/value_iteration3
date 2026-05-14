function layout = vi_matlab_layout()
%VI_MATLAB_LAYOUT Canonical directory layout for the MATLAB subtree.

    root = fileparts(mfilename('fullpath'));

    layout = struct();
    layout.root = root;

    layout.src = fullfile(root, 'src');
    layout.src_common = fullfile(layout.src, 'common');
    layout.src_cpu = fullfile(layout.src, 'cpu');
    layout.src_cpu_frontier = fullfile(layout.src_cpu, 'frontier');
    layout.src_cpu_reference = fullfile(layout.src_cpu, 'reference');
    layout.src_cuda = fullfile(layout.src, 'cuda');
    layout.src_fpga = fullfile(layout.src, 'fpga');
    layout.src_fpga_soc = fullfile(layout.src_fpga, 'soc');
    layout.src_fpga_stream = fullfile(layout.src_fpga, 'stream');
    layout.src_shared = fullfile(layout.src, 'shared');
    layout.src_shared_bitboard = fullfile(layout.src_shared, 'bitboard');

    layout.workflows = fullfile(root, 'workflows');
    layout.workflows_benchmarks = fullfile(layout.workflows, 'benchmarks');
    layout.workflows_benchmarks_codegen = fullfile(layout.workflows_benchmarks, 'codegen');
    layout.workflows_validation = fullfile(layout.workflows, 'validation');
    layout.workflows_validation_cosim = fullfile(layout.workflows_validation, 'cosim');
    layout.workflows_validation_fixedpoint = fullfile(layout.workflows_validation, 'fixedpoint');
    layout.workflows_validation_tests = fullfile(layout.workflows_validation, 'tests');

    layout.platforms = fullfile(root, 'platforms');
    layout.platforms_fpga = fullfile(layout.platforms, 'fpga');
    layout.platforms_fpga_board_support = fullfile(layout.platforms_fpga, 'board_support');
    layout.platforms_fpga_export = fullfile(layout.platforms_fpga, 'export');
    layout.platforms_fpga_model = fullfile(layout.platforms_fpga, 'model');
    layout.platforms_fpga_soc = fullfile(layout.platforms_fpga, 'soc');

    layout.artifacts = fullfile(root, 'artifacts');
    layout.artifacts_benchmarks = fullfile(layout.artifacts, 'benchmarks');
    layout.artifacts_benchmarks_codegen = fullfile(layout.artifacts_benchmarks, 'codegen_build');
    layout.artifacts_benchmarks_results = fullfile(layout.artifacts_benchmarks, 'results');
    layout.artifacts_build = fullfile(layout.artifacts, 'build');
    layout.artifacts_build_repo_ip = fullfile(layout.artifacts_build, 'repo_ip_prj');
    layout.artifacts_cosim = fullfile(layout.artifacts, 'cosim');
    layout.artifacts_derived = fullfile(layout.artifacts, 'derived');
    layout.artifacts_opcount_build = fullfile(layout.artifacts, 'opcount_build');
    layout.artifacts_slprj = fullfile(layout.artifacts, 'slprj');
    layout.artifacts_soc = fullfile(layout.artifacts, 'soc');
end
