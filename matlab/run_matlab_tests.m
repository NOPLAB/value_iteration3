function results = run_matlab_tests()
%RUN_MATLAB_TESTS Run the MATLAB unit test suite for this project.

    layout = vi_matlab_layout();
    original_path = path();
    cleanup = onCleanup(@() path(original_path)); %#ok<NASGU>

    setup_matlab_paths('src', 'tests');

    results = runtests(layout.workflows_validation_tests, 'IncludeSubfolders', true);
    if any([results.Failed])
        error('run_matlab_tests:Failed', '%d MATLAB tests failed.', ...
            nnz([results.Failed]));
    end
end
