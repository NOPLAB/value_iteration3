function cases = bench_cases()
%BENCH_CASES Define the matrix of benchmark cases for benchmark_vi.

    % Sizes are capped at 32 because vi_full_reference is pure-MATLAB triple-loop
    % and 64x64 takes ~30 min per map type. Extend if you can wait.
    cases = struct('name', {}, 'map_x', {}, 'map_y', {}, 'type', {}, 'opts', {});
    for sz = [8, 16, 32]
        for types = {'empty', 'obstacle', 'sentinel', 'random'}
            t = types{1};
            if strcmp(t, 'random')
                opt = struct('density', 0.15, 'seed', 42);
            else
                opt = struct();
            end
            c = struct( ...
                'name',  sprintf('%s_%d', t, sz), ...
                'map_x', sz, ...
                'map_y', sz, ...
                'type',  t, ...
                'opts',  opt);
            cases(end + 1) = c; %#ok<AGROW>
        end
    end
end
