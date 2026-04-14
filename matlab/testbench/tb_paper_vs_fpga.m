function tb_paper_vs_fpga()
%TB_PAPER_VS_FPGA Report percentage differences between paper and FPGA models.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));

    map_x = 8;
    map_y = 8;
    [value, penalty, ~, ~, goal_mask] = gen_test_map(map_x, map_y, 'obstacle');
    trans = gen_transitions('paper_mc');

    [paper_value, paper_action, paper_sweeps, paper_delta] = vi_full_reference( ...
        value, penalty, goal_mask, trans, map_x, map_y, 0, 15);

    fpga_value = value;
    fpga_sweeps = 0;
    fpga_delta = inf;
    for sweep = 1:15
        [fpga_value, delta0] = vi_sweep_stream_algo(fpga_value, fpga_value, ...
            penalty, goal_mask, trans, map_x, map_y, 0);
        [fpga_value, delta1] = vi_sweep_stream_algo(fpga_value, fpga_value, ...
            penalty, goal_mask, trans, map_x, map_y, 1);
        fpga_sweeps = sweep;
        fpga_delta = max(delta0, delta1);
        if fpga_delta == 0
            break;
        end
    end
    fpga_action = compute_action_table_reference(fpga_value, penalty, ...
        goal_mask, trans, map_x, map_y);

    free_mask = ~goal_mask & repmat(penalty ~= double(vi_params().PENALTY_OBSTACLE), [1, 1, vi_params().N_THETA]);
    total_states = nnz(free_mask);

    value_diff_mask = (paper_value ~= fpga_value) & free_mask;
    action_diff_mask = (paper_action ~= fpga_action) & free_mask;

    value_diff_pct = 100 * nnz(value_diff_mask) / total_states;
    action_diff_pct = 100 * nnz(action_diff_mask) / total_states;

    denom = max(paper_value(free_mask), 1);
    mean_abs_pct = 100 * mean(abs(fpga_value(free_mask) - paper_value(free_mask)) ./ denom);

    fprintf('paper_vs_fpga.value_mismatch_pct=%.4f%%\n', value_diff_pct);
    fprintf('paper_vs_fpga.action_mismatch_pct=%.4f%%\n', action_diff_pct);
    fprintf('paper_vs_fpga.mean_abs_value_pct=%.4f%%\n', mean_abs_pct);
    fprintf('paper_vs_fpga.paper_sweeps=%d final_delta=%.0f\n', paper_sweeps, paper_delta);
    fprintf('paper_vs_fpga.fpga_sweeps=%d final_delta=%.0f\n', fpga_sweeps, fpga_delta);

    assert(isfinite(value_diff_pct) && isfinite(action_diff_pct) && isfinite(mean_abs_pct), ...
        'Percentage metrics must be finite');
end
