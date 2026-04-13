function p = vi_params()
%VI_PARAMS Shared constants for the MATLAB streaming VI kernel.
%   Mirrors fpga/hls/stream/src/vi_stream_types.h.

    p.N_ACTIONS       = 6;
    p.N_THETA         = 60;
    p.HALO_MAX        = 6;
    p.WINDOW_ROWS     = 2 * p.HALO_MAX + 1;   % 13
    p.STRIP_W_MAX     = 145;
    p.BUF_W           = p.STRIP_W_MAX + 2 * p.HALO_MAX;  % 157
    p.TRANS_TABLE_SIZE = p.N_ACTIONS * p.N_THETA;  % 360

    % Sentinel values (uint16)
    p.MAX_VALUE        = uint16(hex2dec('FFFF'));  % 65535
    p.PENALTY_OBSTACLE = uint16(hex2dec('FFFF'));  % 65535
    p.PENALTY_GOAL     = uint16(hex2dec('FFFE'));  % 65534
end
