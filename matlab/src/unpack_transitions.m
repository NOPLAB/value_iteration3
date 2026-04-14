function model = unpack_transitions(trans)
%UNPACK_TRANSITIONS Decode packed uint32 transition table.

    p = vi_params();
    model.n_outcomes = zeros(p.N_ACTIONS, p.N_THETA);
    model.dix = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
    model.diy = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
    model.dit = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
    model.prob = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);

    for a = 1:p.N_ACTIONS
        for it = 1:p.N_THETA
            base = ((a - 1) * p.N_THETA + (it - 1)) * p.TRANS_WORD_STRIDE + 1;
            n_out = double(trans(base));
            model.n_outcomes(a, it) = n_out;
            for k = 1:n_out
                word0 = trans(base + 2 * k - 1);
                model.dix(a, it, k) = double(typecast(uint8(bitand(word0, 255)), 'int8'));
                model.diy(a, it, k) = double(typecast(uint8(bitand(bitshift(word0, -8), 255)), 'int8'));
                model.dit(a, it, k) = double(typecast(uint8(bitand(bitshift(word0, -16), 255)), 'int8'));
                model.prob(a, it, k) = double(trans(base + 2 * k));
            end
        end
    end
end
