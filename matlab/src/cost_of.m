function c = cost_of(nv, np_raw)
%COST_OF Compute traversal cost for one neighbor.
%   Matches fpga/hls/stream/src/compute_row.cpp:cost_of().
%   All arithmetic in double (Phase A). HDL Coder target.

    MAX_VALUE        = 65535;
    PENALTY_OBSTACLE = 65535;
    PENALTY_GOAL     = 65534;

    if nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE
        c = MAX_VALUE;
        return;
    end

    if np_raw == PENALTY_GOAL
        np = 0;
    else
        np = np_raw;
    end

    s = nv + np;
    if s >= MAX_VALUE
        c = MAX_VALUE - 1;
    else
        c = s;
    end
end
