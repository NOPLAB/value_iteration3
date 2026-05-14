function cases = bench_cases()
%BENCH_CASES Define the matrix of benchmark cases for benchmark_vi.

    % Sizes are capped at 32 because vi_full_reference is pure-MATLAB triple-loop
    % and 64x64 takes ~30 min per map type. Extend if you can wait.
    sizes = [8, 16, 32];
    types = {'empty', 'obstacle', 'sentinel', 'random'};

    rand_opts = struct('density', 0.15, 'seed', 42);
    empty_opts = struct();

    n = numel(sizes) * numel(types);
    cases = repmat(struct('name', '', 'map_x', 0, 'map_y', 0, ...
        'type', '', 'opts', struct()), 1, n);

    k = 1;
    for i = 1:numel(sizes)
        for j = 1:numel(types)
            sz = sizes(i);
            t  = types{j};
            if strcmp(t, 'random')
                opt = rand_opts;
            else
                opt = empty_opts;
            end
            cases(k) = struct( ...
                'name',  sprintf('%s_%d', t, sz), ...
                'map_x', sz, ...
                'map_y', sz, ...
                'type',  t, ...
                'opts',  opt);
            k = k + 1;
        end
    end
end
