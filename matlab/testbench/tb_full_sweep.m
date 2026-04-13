function tb_full_sweep()
%TB_FULL_SWEEP Full kernel test comparing MATLAB algo vs C reference.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);

    test_cases = {
        struct('name','empty_8x8',    'mx',8,  'my',8,  'type','empty')
        struct('name','empty_32x32',  'mx',32, 'my',32, 'type','empty')
        struct('name','obstacle_16x16','mx',16,'my',16, 'type','obstacle')
        struct('name','sentinel_8x8', 'mx',8,  'my',8,  'type','sentinel')
    };

    trans = gen_transitions('trivial');

    for tc = 1:numel(test_cases)
        t = test_cases{tc};
        fprintf('  Test: %s ... ', t.name);

        [value, penalty, ~, ~] = gen_test_map(t.mx, t.my, t.type);

        % Run C reference
        [ref_out, ~] = run_c_reference(value, penalty, trans, ...
                                        t.mx, t.my, 0, 200);

        % Run MATLAB kernel (same number of sweeps as reference for comparison)
        % We run a fixed number of sweeps to compare intermediate state
        ml_value = value;
        for sweep = 1:50
            [ml_value, delta0] = vi_sweep_stream_algo(ml_value, ml_value, ...
                                                       penalty, trans, ...
                                                       t.mx, t.my, 0);
            [ml_value, delta1] = vi_sweep_stream_algo(ml_value, ml_value, ...
                                                       penalty, trans, ...
                                                       t.mx, t.my, 1);
            if max(delta0, delta1) == 0
                break;
            end
        end

        % Compare converged values: both should converge to same result.
        % After full convergence, reachable cells must have identical
        % optimal cost values between MATLAB and C reference.
        ml_reachable = (ml_value < MV);
        ref_reachable = (ref_out < MV);
        assert(isequal(ml_reachable, ref_reachable), ...
            [t.name ': reachability mismatch']);

        % Verify actual converged values match (not just reachability)
        reachable_mask = ref_reachable;
        ml_vals = ml_value(reachable_mask);
        ref_vals = ref_out(reachable_mask);
        assert(isequal(ml_vals, ref_vals), ...
            [t.name ': value mismatch on reachable cells']);

        % Goal cells must be 0 in both
        goal_mask = (penalty == double(p.PENALTY_GOAL));
        for it = 1:p.N_THETA
            ml_slice = ml_value(:,:,it);
            ref_slice = ref_out(:,:,it);
            assert(all(ml_slice(goal_mask) == 0), [t.name ': MATLAB goal not 0']);
            assert(all(ref_slice(goal_mask) == 0), [t.name ': Ref goal not 0']);
        end

        fprintf('PASSED\n');
    end

    disp('tb_full_sweep: ALL PASSED');
end
