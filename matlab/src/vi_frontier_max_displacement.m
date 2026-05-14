function [mx, my, mt] = vi_frontier_max_displacement(transitions)
%VI_FRONTIER_MAX_DISPLACEMENT Scan transition table for max |dix|, |diy|, |dit|.
%   Used by the frontier-VI variants to size the predecessor cone for expand().
    tm = coerce_transition_model(transitions);
    nA = size(tm.n_outcomes, 1);
    nT = size(tm.n_outcomes, 2);
    mx = 0; my = 0; mt = 0;
    for a = 1:nA
        for it = 1:nT
            n_out = tm.n_outcomes(a, it);
            if n_out <= 0; continue; end
            ddx = abs(tm.dix(a, it, 1:n_out));
            ddy = abs(tm.diy(a, it, 1:n_out));
            ddt = abs(tm.dit(a, it, 1:n_out));
            mx = max(mx, max(ddx(:)));
            my = max(my, max(ddy(:)));
            mt = max(mt, max(ddt(:)));
        end
    end
    mx = double(mx); my = double(my); mt = double(mt);
end
