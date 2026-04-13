function trans = gen_transitions(mode)
%GEN_TRANSITIONS Generate transition table as uint32 array [N_ACTIONS*N_THETA x 1].
%   mode: 'trivial' — action 0 = dix+1, action 1 = dix-1, rest no-op.
%         'full'    — 6 actions with heading-dependent dx/dy/dtheta.
%
%   Each entry packs (dix, diy, dit) as:
%     byte0 = dix (int8), byte1 = diy (int8), byte2 = dit (int8)
%
%   Returns: trans — uint32 [360 x 1]

    p = vi_params();
    trans = zeros(p.N_ACTIONS * p.N_THETA, 1, 'uint32');

    if strcmp(mode, 'trivial')
        for it = 1:p.N_THETA
            % Action 0: dix=+1, diy=0, dit=0
            trans((0) * p.N_THETA + it) = uint32(1);  % 0x00000001
            % Action 1: dix=-1, diy=0, dit=0
            trans((1) * p.N_THETA + it) = uint32(255); % 0x000000FF = int8(-1) as uint8
            % Actions 2-5: no-op (all zeros)
        end
    elseif strcmp(mode, 'full')
        % Full 6-action model with heading-dependent offsets.
        % Resolution: 0.05 m/cell. Forward speed: 0.3 m → 6 cells.
        % dtheta: ±3 indices (±18 deg).
        resolution = 0.05;
        forward_dist = 0.3;
        cells = round(forward_dist / resolution);  % 6
        dt_turn = 3;  % indices for ±18 deg turn

        for it = 1:p.N_THETA
            theta = (it - 1) * (2 * pi / p.N_THETA);
            dix_fwd = round(cells * cos(theta));
            diy_fwd = round(cells * sin(theta));
            dix_bwd = -dix_fwd;
            diy_bwd = -diy_fwd;

            % Action 0: forward
            trans((0)*p.N_THETA + it) = pack_trans(dix_fwd, diy_fwd, 0);
            % Action 1: backward
            trans((1)*p.N_THETA + it) = pack_trans(dix_bwd, diy_bwd, 0);
            % Action 2: turn left — pure rotation, no spatial displacement
            trans((2)*p.N_THETA + it) = pack_trans(0, 0, dt_turn);
            % Action 3: turn right — pure rotation, no spatial displacement
            trans((3)*p.N_THETA + it) = pack_trans(0, 0, -dt_turn);
            % Action 4: forward-left (spatial move + theta change)
            trans((4)*p.N_THETA + it) = pack_trans(dix_fwd, diy_fwd, dt_turn);
            % Action 5: forward-right (spatial move + theta change)
            trans((5)*p.N_THETA + it) = pack_trans(dix_fwd, diy_fwd, -dt_turn);
        end
    else
        error('Unknown mode: %s', mode);
    end
end

function w = pack_trans(dix, diy, dit)
%PACK_TRANS Pack (dix, diy, dit) into uint32 matching HLS format.
    b0 = typecast(int8(dix), 'uint8');
    b1 = typecast(int8(diy), 'uint8');
    b2 = typecast(int8(dit), 'uint8');
    w = uint32(b0) + bitshift(uint32(b1), 8) + bitshift(uint32(b2), 16);
end
