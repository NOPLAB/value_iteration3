function p = vi_params()
%VI_PARAMS Shared constants for the MATLAB streaming VI kernel.
%   Mirrors fpga/hls/stream/src/vi_stream_types.h.

    p.N_ACTIONS       = 6;
    p.N_THETA         = 60;
    p.HALO_MAX        = 6;
    p.WINDOW_ROWS     = 2 * p.HALO_MAX + 1;   % 13
    p.STRIP_W_MAX     = 145;
    p.BUF_W           = p.STRIP_W_MAX + 2 * p.HALO_MAX;  % 157
    p.RESOLUTION_XY_BIT = 6;
    p.RESOLUTION_T_BIT  = 6;
    p.PROB_BASE_BIT     = 2 * p.RESOLUTION_XY_BIT + p.RESOLUTION_T_BIT;  % 18
    p.PROB_BASE         = 2 ^ p.PROB_BASE_BIT;  % 262144

    % Paper Table 1 action set/order
    p.ACTION_FW  = [0.3, -0.2, 0.0, 0.2, 0.0, 0.2];
    p.ACTION_ROT = [0.0,  0.0, -20.0, -20.0, 20.0, 20.0];

    % Maximum unique Monte Carlo outcomes across all action/theta pairs
    p.MAX_OUTCOMES      = 10;
    p.TRANS_WORD_STRIDE = 1 + 2 * p.MAX_OUTCOMES;
    p.TRANS_TABLE_SIZE  = p.N_ACTIONS * p.N_THETA * p.TRANS_WORD_STRIDE;

    % Sentinel values (uint16)
    p.MAX_VALUE        = uint16(hex2dec('FFFF'));  % 65535
    p.PENALTY_OBSTACLE = uint16(hex2dec('FFFF'));  % 65535
    p.PENALTY_GOAL     = uint16(hex2dec('FFFE'));  % retained for compatibility
end
