classdef TestFrontierEquivalence < TestBase
%TESTFRONTIEREQUIVALENCE Bit-exact equivalence of frontier-VI variants vs reference.

    properties (TestParameter)
        bench_case = struct( ...
            'empty_8',     struct('map_x',  8, 'map_y',  8, 'type', 'empty',    'opts', struct()), ...
            'obstacle_8',  struct('map_x',  8, 'map_y',  8, 'type', 'obstacle', 'opts', struct()), ...
            'sentinel_8',  struct('map_x',  8, 'map_y',  8, 'type', 'sentinel', 'opts', struct()), ...
            'random_8',    struct('map_x',  8, 'map_y',  8, 'type', 'random',   'opts', struct('density', 0.15, 'seed', 42)), ...
            'empty_16',    struct('map_x', 16, 'map_y', 16, 'type', 'empty',    'opts', struct()), ...
            'obstacle_16', struct('map_x', 16, 'map_y', 16, 'type', 'obstacle', 'opts', struct()), ...
            'random_16',   struct('map_x', 16, 'map_y', 16, 'type', 'random',   'opts', struct('density', 0.15, 'seed', 42)));
    end

    methods (Test)
        function testFrontier2dMatchesReference(testCase, bench_case)
            [v_ref, v_frontier] = local_run_pair( ...
                bench_case, @vi_frontier_2d);
            testCase.verifyEqual(v_frontier, v_ref);
        end

        function testFrontier3dMatchesReference(testCase, bench_case)
            [v_ref, v_frontier] = local_run_pair( ...
                bench_case, @vi_frontier_3d);
            testCase.verifyEqual(v_frontier, v_ref);
        end

        function testFrontierStackMatchesReference(testCase, bench_case)
            [v_ref, v_frontier] = local_run_pair( ...
                bench_case, @vi_frontier_stack);
            testCase.verifyEqual(v_frontier, v_ref);
        end
    end
end

function [v_ref, v_frontier] = local_run_pair(bench_case, frontier_fn)
    [value0, penalty, ~, ~, goal_mask] = gen_test_map( ...
        bench_case.map_x, bench_case.map_y, bench_case.type, bench_case.opts);
    transitions = gen_transitions('paper_mc');
    max_iters = 400;

    v_ref = vi_full_reference(value0, penalty, goal_mask, transitions, ...
        bench_case.map_x, bench_case.map_y, 0, max_iters);

    v_frontier = frontier_fn(value0, penalty, goal_mask, transitions, ...
        bench_case.map_x, bench_case.map_y, max_iters);
end
