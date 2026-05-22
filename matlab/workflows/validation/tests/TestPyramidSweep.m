classdef TestPyramidSweep < TestBase
%TESTPYRAMIDSWEEP Equivalence tests for 2x2 aggregation pyramid VI.

    properties (TestParameter)
        bench_case = struct( ...
            'empty_8',    struct('map_x', 8, 'map_y', 8, 'type', 'empty',    'opts', struct()), ...
            'obstacle_8', struct('map_x', 8, 'map_y', 8, 'type', 'obstacle', 'opts', struct()), ...
            'random_8',   struct('map_x', 8, 'map_y', 8, 'type', 'random',   'opts', struct('density', 0.15, 'seed', 42)));
    end

    methods (Test)
        function testPyramidSweepMatchesReference(testCase, bench_case)
            [value0, penalty, ~, ~, goal_mask] = gen_test_map( ...
                bench_case.map_x, bench_case.map_y, bench_case.type, ...
                bench_case.opts);
            transitions = gen_transitions('paper_mc');
            max_sweeps = 400;

            v_ref = vi_full_reference(value0, penalty, goal_mask, transitions, ...
                bench_case.map_x, bench_case.map_y, 0, max_sweeps);

            v_pyramid = vi_pyramid_sweep(value0, penalty, goal_mask, transitions, ...
                bench_case.map_x, bench_case.map_y, 0, max_sweeps, 2, 8, max_sweeps);

            testCase.verifyEqual(v_pyramid, v_ref);
        end
    end
end
