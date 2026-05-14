function model = coerce_transition_model(transitions)
%COERCE_TRANSITION_MODEL Accept struct, packed table, or deterministic deltas.
%#codegen

    if isstruct(transitions)
        model = transitions;
        return;
    end

    if coder.target('MATLAB')
        model = coerce_transition_model_matlab(transitions);
    else
        % Codegen path: entry wrappers always pre-unpack to a struct, so any
        % non-struct input here is a contract violation. Returning unchanged
        % keeps the generated code well-typed; the algorithm will then fail
        % loudly when it tries to index struct fields on a non-struct.
        model = transitions;
    end
end

function model = coerce_transition_model_matlab(transitions)
% MATLAB-only path with persistent cache and polymorphic dispatch. Excluded
% from MATLAB Coder via the coder.target gate in the caller.

    p = vi_params();
    persistent last_packed last_model

    if isvector(transitions)
        if ~isempty(last_packed) && isequal(last_packed, transitions)
            model = last_model;
            return;
        end
        model = unpack_transitions(transitions);
        last_packed = transitions;
        last_model = model;
        return;
    end

    if ndims(transitions) == 3 && size(transitions, 3) == 3
        model.n_outcomes = ones(p.N_ACTIONS, p.N_THETA);
        model.dix = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
        model.diy = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
        model.dit = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
        model.prob = zeros(p.N_ACTIONS, p.N_THETA, p.MAX_OUTCOMES);
        model.dix(:, :, 1) = transitions(:, :, 1);
        model.diy(:, :, 1) = transitions(:, :, 2);
        model.dit(:, :, 1) = transitions(:, :, 3);
        model.prob(:, :, 1) = p.PROB_BASE;
        return;
    end

    error('Unsupported transition representation.');
end
