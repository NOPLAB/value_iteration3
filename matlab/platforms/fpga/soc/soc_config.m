function cfg = soc_config()
%SOC_CONFIG SoC Builder configuration for Ultra96-V2.

    layout = vi_matlab_layout();

    cfg.board = 'Avnet Ultra96-V2';  % Or custom BSP name
    cfg.device = 'xczu3eg-sbva484-1-i';
    cfg.vivado_version = '2025.2';
    cfg.clock_freq_mhz = 100;
    cfg.workflow = 'IP Core Generation';
    cfg.build_root = fullfile(layout.artifacts_soc, 'b');
    cfg.project_dirname = 'p';
    cfg.board_plugin_package = 'Ultra96V2';
    cfg.board_support_root = layout.platforms_fpga_board_support;
    cfg.model_dir = layout.platforms_fpga_model;

    % The real target is Ultra96-V2. Keep fallback opt-in because building for a
    % different board produces artifacts that are not deployable to hardware.
    cfg.target_platform_candidates = {cfg.board};
    cfg.allow_target_platform_fallback = false;
    cfg.fallback_target_platform_candidates = { ...
        'Xilinx Zynq UltraScale+ MPSoC ZCU102 Evaluation Kit' ...
    };

    % Prefer memory-mapped reference designs because this kernel reads and writes DDR.
    cfg.reference_design_candidates = { ...
        'Default system with External LPDDR4 Memory Access', ...
        'Default system with External DDR4 Memory Access', ...
        'Default system with External DDR3 Memory Access', ...
        'Default system' ...
    };
    cfg.reference_design_path = cfg.board_support_root;
    cfg.reference_design_tool_version = cfg.vivado_version;

    cfg.allow_unsupported_tool_version = true;
    cfg.ignore_tool_version_mismatch = true;
    cfg.run_external_build = false;
    cfg.run_model_analyzer = false;
    cfg.generate_software_interface = false;
    cfg.generate_software_interface_model = false;
    cfg.generate_host_interface_script = false;
    cfg.max_num_cores_for_build = 'synthesis tool default';

    % AXI Interface mapping
    cfg.axi_master = {
        struct('name', 'gmem0', 'port', 'HP0', 'direction', 'ReadWrite', ...
               'data_width', 128, 'purpose', 'value_table write')
        struct('name', 'gmem1', 'port', 'HP1', 'direction', 'ReadOnly', ...
               'data_width', 128, 'purpose', 'penalty_table + trans_table')
        struct('name', 'gmem2', 'port', 'HP2', 'direction', 'ReadOnly', ...
               'data_width', 128, 'purpose', 'value_table read')
    };

    cfg.axi_slave = struct('name', 'ctrl', 'port', 'GP0', ...
                           'purpose', 'control registers');

    % CU configuration
    cfg.num_cu = 2;
    cfg.cu_names = {'vi_sweep_cu0', 'vi_sweep_cu1'};
end
