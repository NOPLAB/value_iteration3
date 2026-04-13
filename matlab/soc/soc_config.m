function cfg = soc_config()
%SOC_CONFIG SoC Builder configuration for Ultra96-V2.

    cfg.board = 'Avnet Ultra96-V2';  % Or custom BSP name
    cfg.device = 'xczu3eg-sbva484-1-i';
    cfg.vivado_version = '2025.2';
    cfg.clock_freq_mhz = 100;

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
