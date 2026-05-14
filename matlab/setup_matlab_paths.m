function added = setup_matlab_paths(varargin)
%SETUP_MATLAB_PATHS Add MATLAB project paths by concern.

    layout = vi_matlab_layout();
    requested = cellfun(@char, varargin, 'UniformOutput', false);
    if isempty(requested)
        requested = {'src'};
    end

    added = {};
    for idx = 1:numel(requested)
        key = lower(string(requested{idx}));
        switch key
            case "src"
                added = add_tree(added, layout.src);
            case "tests"
                added = add_tree(added, layout.workflows_validation_tests);
            case "validation"
                added = add_tree(added, layout.workflows_validation);
            case "bench"
                added = add_tree(added, layout.workflows_benchmarks);
            case "fpga-export"
                added = add_dir(added, layout.platforms_fpga_export);
                added = add_dir(added, layout.platforms_fpga_model);
            case "soc"
                added = add_tree(added, layout.platforms_fpga_soc);
                added = add_dir(added, layout.platforms_fpga_model);
                added = add_tree(added, layout.platforms_fpga_board_support);
            case "board-support"
                added = add_tree(added, layout.platforms_fpga_board_support);
            case "all"
                added = add_tree(added, layout.src);
                added = add_tree(added, layout.workflows);
                added = add_tree(added, layout.platforms_fpga);
            otherwise
                error('setup_matlab_paths:UnknownKey', ...
                    'Unknown path group: %s', requested{idx});
        end
    end
end

function added = add_tree(added, root_dir)
    if ~exist(root_dir, 'dir')
        return;
    end
    path_str = genpath(root_dir);
    if isempty(path_str)
        return;
    end
    addpath(path_str);
    added = [added, split_path(path_str)]; %#ok<AGROW>
end

function added = add_dir(added, root_dir)
    if ~exist(root_dir, 'dir')
        return;
    end
    addpath(root_dir);
    added{end + 1} = root_dir; %#ok<AGROW>
end

function parts = split_path(path_str)
    parts = regexp(path_str, pathsep, 'split');
    parts = parts(~cellfun('isempty', parts));
end
