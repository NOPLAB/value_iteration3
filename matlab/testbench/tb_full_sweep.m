function tb_full_sweep()
%TB_FULL_SWEEP Compare the FPGA MATLAB model against the paper reference.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);

    test_cases = {
        struct('name', 'empty_6x6',      'mx', 6, 'my', 6, 'type', 'empty')
        struct('name', 'obstacle_8x8',   'mx', 8, 'my', 8, 'type', 'obstacle')
    };

    trans = gen_transitions('paper_mc');

    for tc = 1:numel(test_cases)
        t = test_cases{tc};
        fprintf('  Test: %s ... ', t.name);

        [value, penalty, ~, ~, goal_mask] = gen_test_map(t.mx, t.my, t.type);
        [ref_out, ref_action, ~, ~] = vi_full_reference(value, penalty, ...
            goal_mask, trans, t.mx, t.my, 0, 15);

        ml_value = value;
        for sweep = 1:15
            [ml_value, delta0] = vi_sweep_stream_algo(ml_value, ml_value, ...
                penalty, goal_mask, trans, t.mx, t.my, 0);
            [ml_value, delta1] = vi_sweep_stream_algo(ml_value, ml_value, ...
                penalty, goal_mask, trans, t.mx, t.my, 1);
            if max(delta0, delta1) == 0
                break;
            end
        end

        ml_action = compute_action_table_reference(ml_value, penalty, ...
            goal_mask, trans, t.mx, t.my);

        free_mask = ~goal_mask & repmat(penalty ~= double(p.PENALTY_OBSTACLE), [1, 1, p.N_THETA]);
        ml_vals = ml_value(free_mask);
        ref_vals = ref_out(free_mask);
        assert(isequal(ml_vals < MV, ref_vals < MV), [t.name ': reachability mismatch']);
        assert(isequal(ml_vals, ref_vals), [t.name ': value mismatch']);
        assert(isequal(ml_action(free_mask), ref_action(free_mask)), [t.name ': action mismatch']);
        assert(all(ml_value(goal_mask) == 0), [t.name ': goal values changed']);

        fprintf('PASSED\n');
    end

    disp('tb_full_sweep: ALL PASSED');
end
