function tb_stream_strip()
%TB_STREAM_STRIP Integration test for stream_strip_algo.
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    p = vi_params();
    MV = double(p.MAX_VALUE);

    map_x = 16; map_y = 16;
    [value, penalty, gx, gy] = gen_test_map(map_x, map_y, 'empty');
    trans = gen_transitions('trivial');

    % Unpack transitions to delta_table [N_ACTIONS, N_THETA, 3]
    delta_table = unpack_transitions(trans, p);

    % Run one strip (covers full map width since 16 < STRIP_W_MAX)
    strip_x0 = 0; strip_w = map_x; cu_id = 0;
    [value_out, strip_delta] = stream_strip_algo(value, value, penalty, ...
                                                  delta_table, map_x, map_y, ...
                                                  strip_x0, strip_w, cu_id);

    % Goal should still be 0
    assert(value_out(gy, gx, 1) == 0, 'Goal value changed');
    % Adjacent cells should be updated
    assert(value_out(gy, gx+1, 1) < MV, 'Adjacent cell not updated');
    assert(strip_delta > 0, 'No delta after first sweep');

    disp('tb_stream_strip: ALL PASSED');
end

function delta_table = unpack_transitions(trans, p)
%UNPACK_TRANSITIONS Convert uint32 flat array to [N_ACTIONS, N_THETA, 3].
    delta_table = zeros(p.N_ACTIONS, p.N_THETA, 3);
    for i = 1:p.TRANS_TABLE_SIZE
        a = floor((i-1) / p.N_THETA) + 1;
        t = mod(i-1, p.N_THETA) + 1;
        w = trans(i);
        delta_table(a, t, 1) = double(typecast(uint8(bitand(w, 255)), 'int8'));
        delta_table(a, t, 2) = double(typecast(uint8(bitand(bitshift(w,-8), 255)), 'int8'));
        delta_table(a, t, 3) = double(typecast(uint8(bitand(bitshift(w,-16), 255)), 'int8'));
    end
end
