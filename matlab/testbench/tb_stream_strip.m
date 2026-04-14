function tb_stream_strip()
%TB_STREAM_STRIP Integration test for stream_strip_algo.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);

    map_x = 16; map_y = 16;
    [value, penalty, goal_x, goal_y, goal_mask] = gen_test_map(map_x, map_y, 'empty');
    trans = gen_transitions('trivial');

    trans_model = unpack_transitions(trans);

    % Run one strip (covers full map width since 16 < STRIP_W_MAX)
    strip_x0 = 0; strip_w = map_x; cu_id = 0;
    [value_out, strip_delta] = stream_strip_algo(value, value, penalty, ...
                                                  goal_mask, trans_model, ...
                                                  map_x, map_y, strip_x0, ...
                                                  strip_w, cu_id);

    goal_theta = find(squeeze(goal_mask(goal_y, goal_x, :)), 1, 'first');

    % Goal states should still be 0.
    assert(all(value_out(goal_mask) == 0), 'Goal value changed');

    boundary_x = [];
    for x = 2:map_x
        if goal_mask(goal_y, x - 1, goal_theta) && ~goal_mask(goal_y, x, goal_theta)
            boundary_x = x;
            break;
        end
    end
    assert(~isempty(boundary_x), 'Failed to find a goal boundary cell');
    assert(value_out(goal_y, boundary_x, goal_theta) == 1, ...
        'Boundary-adjacent cell not updated to step cost 1');
    assert(strip_delta > 0, 'No delta after first sweep');

    disp('tb_stream_strip: ALL PASSED');
end
