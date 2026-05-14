function results = run_matlab_tests()
%RUN_MATLAB_TESTS Run the MATLAB unit test suite for this project.

    root_dir = fileparts(mfilename('fullpath'));
    original_path = path();
    cleanup = onCleanup(@() path(original_path)); %#ok<NASGU>

    addpath(fullfile(root_dir, 'src'));
    addpath(fullfile(root_dir, 'src', 'bitboard'));
    addpath(fullfile(root_dir, 'test'));

    results = runtests(fullfile(root_dir, 'test'), 'IncludeSubfolders', true);
    if any([results.Failed])
        error('run_matlab_tests:Failed', '%d MATLAB tests failed.', ...
            nnz([results.Failed]));
    end
end
