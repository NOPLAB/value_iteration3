function create_model()
%CREATE_MODEL Build the Simulink model for the streaming VI kernel.
%   Creates vi_sweep_stream_matlab.slx with:
%   - Algorithm subsystem referencing the .m functions
%   - HDL Coder configuration
%   - SoC Blockset annotations for later IP generation

    model_name = 'vi_sweep_stream_matlab';
    model_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(model_dir, '..', 'src'));

    % Close if already open
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    % Create new model
    new_system(model_name);
    open_system(model_name);

    % Set solver to fixed-step (required for HDL Coder)
    set_param(model_name, 'Solver', 'FixedStepDiscrete');
    set_param(model_name, 'FixedStep', '1');
    set_param(model_name, 'StopTime', 'inf');

    % Add Algorithm subsystem (MATLAB Function block)
    algo_path = [model_name '/Algorithm'];
    add_block('simulink/User-Defined Functions/MATLAB Function', algo_path);

    % Configure the MATLAB Function block with the algorithm
    % The function references vi_sweep_stream_algo.m
    mfb = get_param(algo_path, 'Object');

    % Set HDL Coder parameters
    hdlset_param(model_name, 'HDLSubsystem', model_name);
    hdlset_param(model_name, 'SynthesisTool', 'Xilinx Vivado');
    hdlset_param(model_name, 'SynthesisToolChipFamily', 'Zynq UltraScale+');
    hdlset_param(model_name, 'SynthesisToolDeviceName', 'xczu3eg');
    hdlset_param(model_name, 'SynthesisToolPackageName', 'sbva484');
    hdlset_param(model_name, 'SynthesisToolSpeedValue', '-1');

    % Save
    save_system(model_name, fullfile(model_dir, [model_name '.slx']));
    fprintf('Model saved: %s.slx\n', model_name);
end
