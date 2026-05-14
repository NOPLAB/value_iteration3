classdef TestAlgorithmUnits < TestBase
%TESTALGORITHMUNITS Unit and component tests for MATLAB VI helpers.

    properties (TestParameter)
        cost_case = struct( ...
            'normal_addition', struct('neighbor_value', 100, ...
                                      'penalty', 50, ...
                                      'expected', 151), ...
            'max_value_neighbor', struct('neighbor_value', 65535, ...
                                         'penalty', 50, ...
                                         'expected', 65535), ...
            'obstacle_penalty', struct('neighbor_value', 100, ...
                                       'penalty', 65535, ...
                                       'expected', 65535), ...
            'goal_penalty', struct('neighbor_value', 100, ...
                                   'penalty', 65534, ...
                                   'expected', 101), ...
            'saturation', struct('neighbor_value', 65000, ...
                                 'penalty', 600, ...
                                 'expected', 65534), ...
            'goal_zero', struct('neighbor_value', 0, ...
                                'penalty', 65534, ...
                                'expected', 1), ...
            'sentinel_pair', struct('neighbor_value', 65535, ...
                                    'penalty', 65535, ...
                                    'expected', 65535));
    end

    methods (Test)
        function testCostOfScenarios(testCase, cost_case)
            actual = cost_of(cost_case.neighbor_value, cost_case.penalty);
            testCase.verifyEqual(actual, cost_case.expected);
        end

        function testLoadRowHandlesInBoundsAndOutOfBounds(testCase)
            p = vi_params();
            max_value = double(p.MAX_VALUE);
            obstacle_penalty = double(p.PENALTY_OBSTACLE);

            map_x = 16;
            map_y = 10;
            strip_x0 = 0;
            strip_w = 16;

            value_table = max_value * ones(map_y, map_x, p.N_THETA);
            penalty_table = zeros(map_y, map_x);
            goal_mask = false(map_y, map_x, p.N_THETA);
            value_table(3, 5, :) = 100;
            penalty_table(3, 5) = 42;

            [val_row, pen_row] = load_row_algo(value_table, penalty_table, ...
                goal_mask, 2, strip_x0, strip_w, map_x, map_y);
            bx = 4 + p.HALO_MAX + 1;
            testCase.verifyEqual(val_row(bx, 1), 100);
            testCase.verifyEqual(pen_row(bx), 42);
            testCase.verifyEqual(val_row(1, 1), max_value);
            testCase.verifyEqual(pen_row(1), obstacle_penalty);

            [val_oob, pen_oob] = load_row_algo(value_table, penalty_table, ...
                goal_mask, -1, strip_x0, strip_w, map_x, map_y);
            testCase.verifyTrue(all(pen_oob == obstacle_penalty));
            testCase.verifyTrue(all(val_oob(:, 1) == max_value));
        end

        function testStoreRowWritesBackModifiedValues(testCase)
            p = vi_params();
            max_value = double(p.MAX_VALUE);

            map_x = 16;
            map_y = 10;
            strip_x0 = 0;
            strip_w = 16;

            value_table = max_value * ones(map_y, map_x, p.N_THETA);
            penalty_table = zeros(map_y, map_x);
            goal_mask = false(map_y, map_x, p.N_THETA);
            value_table(3, 5, :) = 100;
            penalty_table(3, 5) = 42;

            [val_row, ~] = load_row_algo(value_table, penalty_table, ...
                goal_mask, 2, strip_x0, strip_w, map_x, map_y);
            val_row(p.HALO_MAX + 1, 1) = 999;
            value_table = store_row_algo(val_row, value_table, ...
                2, strip_x0, strip_w, map_x);

            testCase.verifyEqual(value_table(3, 1, 1), 999);
            testCase.verifyEqual(value_table(3, 5, 1), 100);
        end

        function testComputeRowUpdatesReachableNeighbor(testCase)
            p = vi_params();
            max_value = double(p.MAX_VALUE);
            obstacle_penalty = double(p.PENALTY_OBSTACLE);

            goal_buf = false(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);
            val_buf = max_value * ones(p.WINDOW_ROWS, p.BUF_W, p.N_THETA);
            pen_buf = obstacle_penalty * ones(p.WINDOW_ROWS, p.BUF_W);

            win_center = p.HALO_MAX + 1;
            bx_goal = 10;
            val_buf(win_center, bx_goal, 1) = 0;
            pen_buf(win_center, bx_goal) = 0;
            goal_buf(win_center, bx_goal, 1) = true;

            pen_buf(win_center, 9) = 0;
            val_buf(win_center, 9, :) = max_value;

            delta_table = zeros(p.N_ACTIONS, p.N_THETA, 3);
            for it = 1:p.N_THETA
                delta_table(1, it, 1) = 1;
                delta_table(2, it, 1) = -1;
            end

            [val_buf_out, row_max_delta] = compute_row_algo(val_buf, pen_buf, ...
                goal_buf, delta_table, win_center, 16, 0);

            testCase.verifyEqual(val_buf_out(win_center, 9, 1), 1);
            testCase.verifyEqual(val_buf_out(win_center, bx_goal, 1), 0);
            testCase.verifyEqual(val_buf_out(win_center, 1, 1), max_value);
            testCase.verifyGreaterThanOrEqual(row_max_delta, 0);
        end

        function testGoalAreaCoversExpectedPoseRange(testCase)
            goal_mask = make_goal_mask(16, 16, struct( ...
                'xy_resolution', 0.05, ...
                'map_origin_x', 0.0, ...
                'map_origin_y', 0.0, ...
                'goal_x', 0.225, ...
                'goal_y', 0.225, ...
                'goal_theta_deg', 90, ...
                'goal_radius_m', 0.30, ...
                'goal_margin_theta_deg', 15));

            testCase.verifyGreaterThan(nnz(goal_mask(:, :, 15)), 1);
            testCase.verifyTrue(goal_mask(5, 5, 14));
            testCase.verifyTrue(goal_mask(5, 5, 17));
            testCase.verifyFalse(goal_mask(5, 5, 13));
            testCase.verifyFalse(goal_mask(5, 5, 18));
            testCase.verifyFalse(goal_mask(16, 16, 15));
        end

        function testStreamStripPreservesGoalAndPropagatesCost(testCase)
            map_x = 16;
            map_y = 16;
            [value, penalty, goal_x, goal_y, goal_mask] = ...
                gen_test_map(map_x, map_y, 'empty');
            trans_model = unpack_transitions(gen_transitions('trivial'));

            [value_out, strip_delta] = stream_strip_algo(value, value, ...
                penalty, goal_mask, trans_model, map_x, map_y, 0, map_x, 0);

            goal_theta = find(squeeze(goal_mask(goal_y, goal_x, :)), ...
                1, 'first');
            boundary_x = [];
            for x = 2:map_x
                if goal_mask(goal_y, x - 1, goal_theta) && ...
                        ~goal_mask(goal_y, x, goal_theta)
                    boundary_x = x;
                    break;
                end
            end

            testCase.verifyTrue(all(value_out(goal_mask) == 0));
            testCase.verifyNotEmpty(boundary_x);
            testCase.verifyEqual(value_out(goal_y, boundary_x, goal_theta), 1);
            testCase.verifyGreaterThan(strip_delta, 0);
        end

        function testPaperMcDitIsRelative(testCase)
            % paper_mc dit must be a signed relative offset, matching the
            % consumer in compute_row_algo / vi_full_reference (nt = it + dit).
            % Regression guard for the absolute-vs-relative interpretation bug.
            clear gen_transitions;
            p = vi_params();
            trans = gen_transitions('paper_mc');
            model = unpack_transitions(trans);

            fw_action = find(p.ACTION_FW > 0 & p.ACTION_ROT == 0, 1);
            bw_action = find(p.ACTION_FW < 0 & p.ACTION_ROT == 0, 1);
            for it = 1:p.N_THETA
                n_fw = model.n_outcomes(fw_action, it);
                fw_dits = squeeze(model.dit(fw_action, it, 1:n_fw));
                testCase.verifyTrue(all(fw_dits == 0), ...
                    sprintf('forward action dit must be 0 at it=%d', it));

                n_bw = model.n_outcomes(bw_action, it);
                bw_dits = squeeze(model.dit(bw_action, it, 1:n_bw));
                testCase.verifyTrue(all(bw_dits == 0), ...
                    sprintf('backward action dit must be 0 at it=%d', it));
            end

            for a = 1:p.N_ACTIONS
                expected = round(p.ACTION_ROT(a) / (360 / p.N_THETA));
                for it = 1:p.N_THETA
                    n = model.n_outcomes(a, it);
                    dits = squeeze(model.dit(a, it, 1:n));
                    testCase.verifyTrue(all(abs(dits - expected) <= 1), ...
                        sprintf('action %d at it=%d: dit %s not within ±1 of expected %d', ...
                            a, it, mat2str(dits(:)'), expected));
                end
            end
        end
    end
end
