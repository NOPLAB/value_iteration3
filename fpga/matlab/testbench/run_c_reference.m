function [value_out, sweeps] = run_c_reference(value, penalty, trans, ...
                                                map_x, map_y, threshold, max_sweeps)
%RUN_C_REFERENCE Run the C reference solver via MEX.
%   value   — double [map_y, map_x, N_THETA] → reshaped to uint16 flat array
%   penalty — double [map_y, map_x] → reshaped to uint16 flat array
%   trans   — uint32 [360 x 1]
%   Returns value_out as double [map_y, map_x, N_THETA]

    p = vi_params();

    % Reshape to C-order flat arrays (row-major: y * map_x * N_THETA + x * N_THETA + t)
    % MATLAB is column-major, so we need to permute and reshape carefully.
    % value is [map_y, map_x, N_THETA] in MATLAB
    % C expects flat[y][x][theta] = flat[y * map_x * N_THETA + x * N_THETA + theta]
    val_perm = permute(value, [3, 2, 1]);  % [N_THETA, map_x, map_y]
    val_flat = uint16(val_perm(:));         % Column-major read = theta-fastest

    % Penalty: [map_y, map_x] → C flat[y * map_x + x]
    pen_perm = permute(penalty, [2, 1]);    % [map_x, map_y]
    pen_flat = uint16(pen_perm(:));

    % Build MEX if not on path
    mex_file = fullfile(fileparts(mfilename('fullpath')), 'vi_reference_mex');
    if ~exist([mex_file '.' mexext], 'file')
        src_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'host', 'src');
        drv_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'driver', 'uio');
        mex_src = fullfile(fileparts(mfilename('fullpath')), 'vi_reference_mex.c');
        ref_src = fullfile(src_dir, 'vi_reference_c.c');
        mex(mex_src, ref_src, ['-I' src_dir], ['-I' drv_dir], ...
            '-output', mex_file);
    end

    [val_out_flat, sweeps] = vi_reference_mex(val_flat, pen_flat, trans, ...
                                               map_x, map_y, threshold, max_sweeps);

    % Reshape back to [map_y, map_x, N_THETA]
    val_out_3d = reshape(double(val_out_flat), [p.N_THETA, map_x, map_y]);
    value_out = permute(val_out_3d, [3, 2, 1]);
end
