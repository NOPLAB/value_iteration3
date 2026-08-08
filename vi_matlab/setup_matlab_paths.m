function setup_matlab_paths(varargin)
%SETUP_MATLAB_PATHS Add MATLAB project paths by concern.

    layout = vi_matlab_layout();
    requested = cellfun(@char, varargin, 'UniformOutput', false);
    if isempty(requested)
        requested = {'src'};
    end

    for idx = 1:numel(requested)
        key = lower(string(requested{idx}));
        switch key
            case "src"
                addpath(genpath(layout.src));
            case "tests"
                addpath(genpath(layout.workflows_validation_tests));
            case "bench"
                addpath(genpath(layout.workflows_benchmarks));
            case "fpga-export"
                addpath(layout.platforms_fpga_export);
                addpath(layout.platforms_fpga_model);
            case "board-support"
                addpath(genpath(layout.platforms_fpga_board_support));
            otherwise
                error('setup_matlab_paths:UnknownKey', ...
                    'Unknown path group: %s', requested{idx});
        end
    end
end
