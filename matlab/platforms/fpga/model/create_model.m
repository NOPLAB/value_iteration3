function create_model()
%CREATE_MODEL Build the Simulink model for the SoC VI kernel.

    model_name = 'vi_sweep_stream_matlab';
    model_dir = fileparts(mfilename('fullpath'));
    setup_matlab_paths('src');

    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    new_system(model_name);
    open_system(model_name);
    set_param(model_name, 'Solver', 'FixedStepDiscrete');
    set_param(model_name, 'FixedStep', '1');
    set_param(model_name, 'StopTime', 'inf');

    create_bus_types(model_name);
    add_top_ports(model_name);
    add_algorithm_subsystem(model_name);
    connect_top_level(model_name);
    configure_hdl(model_name);

    save_system(model_name, fullfile(model_dir, [model_name '.slx']));
    fprintf('Model saved: %s.slx\n', model_name);
end

function create_bus_types(model_name)
    assignin('base', 'ReadControlS2MBusObj', make_bus({ ...
        make_elem('rd_aready', 'boolean'), ...
        make_elem('rd_dvalid', 'boolean') ...
    }));
    assignin('base', 'ReadControlM2SBusObj', make_bus({ ...
        make_elem('rd_addr', 'uint32'), ...
        make_elem('rd_len', 'uint32'), ...
        make_elem('rd_avalid', 'boolean'), ...
        make_elem('rd_dready', 'boolean') ...
    }));
    assignin('base', 'WriteControlS2MBusObj', make_bus({ ...
        make_elem('wr_ready', 'boolean'), ...
        make_elem('wr_bvalid', 'boolean'), ...
        make_elem('wr_complete', 'boolean') ...
    }));
    assignin('base', 'WriteControlM2SBusObj', make_bus({ ...
        make_elem('wr_addr', 'uint32'), ...
        make_elem('wr_len', 'uint32'), ...
        make_elem('wr_valid', 'boolean') ...
    }));
end

function bus = make_bus(elements)
    bus = Simulink.Bus;
    bus.Elements = [elements{:}];
end

function elem = make_elem(name, data_type)
    elem = Simulink.BusElement;
    elem.Name = name;
    elem.DataType = data_type;
    elem.Dimensions = 1;
end

function add_top_ports(model_name)
    ports = { ...
        struct('name', 'gmem2_rdData',   'kind', 'In1',  'pos', [50 70 80 90],   'type', 'uint32')
        struct('name', 'gmem2_rdCtrlIn', 'kind', 'In1',  'pos', [50 120 80 140], 'type', 'Bus: ReadControlS2MBusObj')
        struct('name', 'gmem1_rdData',   'kind', 'In1',  'pos', [50 190 80 210], 'type', 'uint32')
        struct('name', 'gmem1_rdCtrlIn', 'kind', 'In1',  'pos', [50 240 80 260], 'type', 'Bus: ReadControlS2MBusObj')
        struct('name', 'gmem0_wrCtrlIn', 'kind', 'In1',  'pos', [50 310 80 330], 'type', 'Bus: WriteControlS2MBusObj')
        struct('name', 'start',          'kind', 'In1',  'pos', [50 390 80 410], 'type', 'boolean')
        struct('name', 'map_x',          'kind', 'In1',  'pos', [50 440 80 460], 'type', 'uint32')
        struct('name', 'map_y',          'kind', 'In1',  'pos', [50 490 80 510], 'type', 'uint32')
        struct('name', 'cu_id',          'kind', 'In1',  'pos', [50 540 80 560], 'type', 'uint32')
        struct('name', 'gmem2_rdCtrlOut','kind', 'Out1', 'pos', [980 100 1010 120], 'type', 'Bus: ReadControlM2SBusObj')
        struct('name', 'gmem1_rdCtrlOut','kind', 'Out1', 'pos', [980 180 1010 200], 'type', 'Bus: ReadControlM2SBusObj')
        struct('name', 'gmem0_wrData',   'kind', 'Out1', 'pos', [980 260 1010 280], 'type', 'uint32')
        struct('name', 'gmem0_wrCtrlOut','kind', 'Out1', 'pos', [980 320 1010 340], 'type', 'Bus: WriteControlM2SBusObj')
        struct('name', 'done',           'kind', 'Out1', 'pos', [980 420 1010 440], 'type', 'boolean')
        struct('name', 'max_delta',      'kind', 'Out1', 'pos', [980 470 1010 490], 'type', 'uint16')
    };

    for i = 1:numel(ports)
        p = ports{i};
        path = [model_name '/' p.name];
        add_block(['simulink/' ternary(strcmp(p.kind, 'In1'), 'Sources', 'Sinks') '/' p.kind], ...
                  path, 'Position', p.pos);
        set_param(path, 'OutDataTypeStr', p.type);
    end
end

function add_algorithm_subsystem(model_name)
    alg = [model_name '/Algorithm'];
    add_block('simulink/Ports & Subsystems/Subsystem', alg, ...
              'Position', [170 60 900 580]);
    set_param(alg, 'TreatAsAtomicUnit', 'on');
    Simulink.SubSystem.deleteContents(alg);

    in_specs = { ...
        {'gmem2_rdData', 'uint32', [35 60 65 80]}
        {'gmem2_rdCtrlIn', 'Bus: ReadControlS2MBusObj', [35 110 65 130]}
        {'gmem1_rdData', 'uint32', [35 170 65 190]}
        {'gmem1_rdCtrlIn', 'Bus: ReadControlS2MBusObj', [35 220 65 240]}
        {'gmem0_wrCtrlIn', 'Bus: WriteControlS2MBusObj', [35 280 65 300]}
        {'start', 'boolean', [35 350 65 370]}
        {'map_x', 'uint32', [35 400 65 420]}
        {'map_y', 'uint32', [35 450 65 470]}
        {'cu_id', 'uint32', [35 500 65 520]}
    };
    out_specs = { ...
        {'gmem2_rdCtrlOut', 'Bus: ReadControlM2SBusObj', [865 110 895 130]}
        {'gmem1_rdCtrlOut', 'Bus: ReadControlM2SBusObj', [865 190 895 210]}
        {'gmem0_wrData', 'uint32', [865 270 895 290]}
        {'gmem0_wrCtrlOut', 'Bus: WriteControlM2SBusObj', [865 330 895 350]}
        {'done', 'boolean', [865 420 895 440]}
        {'max_delta', 'uint16', [865 470 895 490]}
    };

    for i = 1:numel(in_specs)
        spec = in_specs{i};
        path = [alg '/' spec{1}];
        add_block('simulink/Sources/In1', path, 'Position', spec{3});
        set_param(path, 'OutDataTypeStr', spec{2});
    end

    for i = 1:numel(out_specs)
        spec = out_specs{i};
        path = [alg '/' spec{1}];
        add_block('simulink/Sinks/Out1', path, 'Position', spec{3});
        set_param(path, 'OutDataTypeStr', spec{2});
    end

    add_block('hwlogicconnlib/SoC Bus Selector', [alg '/Gmem2ReadSelector'], ...
              'Position', [120 100 220 150]);
    set_param([alg '/Gmem2ReadSelector'], 'Protocol', 'Random access read', 'ctrltype', 'Valid');
    add_block('hwlogicconnlib/SoC Bus Selector', [alg '/Gmem1ReadSelector'], ...
              'Position', [120 210 220 260]);
    set_param([alg '/Gmem1ReadSelector'], 'Protocol', 'Random access read', 'ctrltype', 'Valid');
    add_block('hwlogicconnlib/SoC Bus Selector', [alg '/Gmem0WriteSelector'], ...
              'Position', [120 290 220 350]);
    set_param([alg '/Gmem0WriteSelector'], 'Protocol', 'Random access write', 'ctrltype', 'Ready');

    add_block('hwlogicconnlib/SoC Bus Creator', [alg '/Gmem2ReadCreator'], ...
              'Position', [650 100 760 150]);
    set_param([alg '/Gmem2ReadCreator'], 'Protocol', 'Random access read', 'ctrltype', 'Ready');
    add_block('hwlogicconnlib/SoC Bus Creator', [alg '/Gmem1ReadCreator'], ...
              'Position', [650 180 760 230]);
    set_param([alg '/Gmem1ReadCreator'], 'Protocol', 'Random access read', 'ctrltype', 'Ready');
    add_block('hwlogicconnlib/SoC Bus Creator', [alg '/Gmem0WriteCreator'], ...
              'Position', [650 320 760 370]);
    set_param([alg '/Gmem0WriteCreator'], 'Protocol', 'Random access write', 'ctrltype', 'Valid');

    kernel = [alg '/Kernel'];
    add_block('simulink/User-Defined Functions/MATLAB Function', kernel, ...
              'Position', [280 80 600 430]);

    rt = sfroot;
    chart = find(rt, '-isa', 'Stateflow.EMChart', 'Path', kernel);
    chart.Script = sprintf([ ...
        'function [gmem2_rd_addr, gmem2_rd_len, gmem2_rd_avalid, gmem2_rd_dready, ...\n', ...
        '          gmem1_rd_addr, gmem1_rd_len, gmem1_rd_avalid, gmem1_rd_dready, ...\n', ...
        '          gmem0_wr_addr, gmem0_wr_len, gmem0_wr_valid, gmem0_wr_data, ...\n', ...
        '          done, max_delta] = fcn(gmem2_rd_data, gmem2_rd_aready, gmem2_rd_dvalid, ...\n', ...
        '          gmem1_rd_data, gmem1_rd_aready, gmem1_rd_dvalid, ...\n', ...
        '          gmem0_wr_ready, gmem0_wr_bvalid, gmem0_wr_complete, ...\n', ...
        '          start, map_x, map_y, cu_id)\n', ...
        '%%#codegen\n', ...
        '[gmem2_rd_addr, gmem2_rd_len, gmem2_rd_avalid, gmem2_rd_dready, ...\n', ...
        ' gmem1_rd_addr, gmem1_rd_len, gmem1_rd_avalid, gmem1_rd_dready, ...\n', ...
        ' gmem0_wr_addr, gmem0_wr_len, gmem0_wr_valid, gmem0_wr_data, ...\n', ...
        ' done, max_delta] = vi_sweep_soc_kernel( ...\n', ...
        '    gmem2_rd_data, gmem2_rd_aready, gmem2_rd_dvalid, ...\n', ...
        '    gmem1_rd_data, gmem1_rd_aready, gmem1_rd_dvalid, ...\n', ...
        '    gmem0_wr_ready, gmem0_wr_bvalid, gmem0_wr_complete, ...\n', ...
        '    start, map_x, map_y, cu_id);\n' ...
    ]);
    configure_kernel_types(chart);

    add_algorithm_lines(alg);
end

function configure_kernel_types(chart)
    names = { ...
        'gmem2_rd_data', 'uint32'
        'gmem2_rd_aready', 'boolean'
        'gmem2_rd_dvalid', 'boolean'
        'gmem1_rd_data', 'uint32'
        'gmem1_rd_aready', 'boolean'
        'gmem1_rd_dvalid', 'boolean'
        'gmem0_wr_ready', 'boolean'
        'gmem0_wr_bvalid', 'boolean'
        'gmem0_wr_complete', 'boolean'
        'start', 'boolean'
        'map_x', 'uint32'
        'map_y', 'uint32'
        'cu_id', 'uint32'
        'gmem2_rd_addr', 'uint32'
        'gmem2_rd_len', 'uint32'
        'gmem2_rd_avalid', 'boolean'
        'gmem2_rd_dready', 'boolean'
        'gmem1_rd_addr', 'uint32'
        'gmem1_rd_len', 'uint32'
        'gmem1_rd_avalid', 'boolean'
        'gmem1_rd_dready', 'boolean'
        'gmem0_wr_addr', 'uint32'
        'gmem0_wr_len', 'uint32'
        'gmem0_wr_valid', 'boolean'
        'gmem0_wr_data', 'uint32'
        'done', 'boolean'
        'max_delta', 'uint16'
    };

    for i = 1:size(names, 1)
        data = find(chart, '-isa', 'Stateflow.Data', 'Name', names{i, 1});
        data.DataType = names{i, 2};
        data.Props.Array.Size = '1';
    end
end

function add_algorithm_lines(alg)
    add_line(alg, 'gmem2_rdCtrlIn/1', 'Gmem2ReadSelector/1');
    add_line(alg, 'gmem1_rdCtrlIn/1', 'Gmem1ReadSelector/1');
    add_line(alg, 'gmem0_wrCtrlIn/1', 'Gmem0WriteSelector/1');

    add_line(alg, 'gmem2_rdData/1', 'Kernel/1');
    add_line(alg, 'Gmem2ReadSelector/1', 'Kernel/2');
    add_line(alg, 'Gmem2ReadSelector/2', 'Kernel/3');

    add_line(alg, 'gmem1_rdData/1', 'Kernel/4');
    add_line(alg, 'Gmem1ReadSelector/1', 'Kernel/5');
    add_line(alg, 'Gmem1ReadSelector/2', 'Kernel/6');

    add_line(alg, 'Gmem0WriteSelector/1', 'Kernel/7');
    add_line(alg, 'Gmem0WriteSelector/2', 'Kernel/8');
    add_line(alg, 'Gmem0WriteSelector/3', 'Kernel/9');

    add_line(alg, 'start/1', 'Kernel/10');
    add_line(alg, 'map_x/1', 'Kernel/11');
    add_line(alg, 'map_y/1', 'Kernel/12');
    add_line(alg, 'cu_id/1', 'Kernel/13');

    add_line(alg, 'Kernel/1', 'Gmem2ReadCreator/1');
    add_line(alg, 'Kernel/2', 'Gmem2ReadCreator/2');
    add_line(alg, 'Kernel/3', 'Gmem2ReadCreator/3');
    add_line(alg, 'Kernel/4', 'Gmem2ReadCreator/4');
    add_line(alg, 'Gmem2ReadCreator/1', 'gmem2_rdCtrlOut/1');

    add_line(alg, 'Kernel/5', 'Gmem1ReadCreator/1');
    add_line(alg, 'Kernel/6', 'Gmem1ReadCreator/2');
    add_line(alg, 'Kernel/7', 'Gmem1ReadCreator/3');
    add_line(alg, 'Kernel/8', 'Gmem1ReadCreator/4');
    add_line(alg, 'Gmem1ReadCreator/1', 'gmem1_rdCtrlOut/1');

    add_line(alg, 'Kernel/9', 'Gmem0WriteCreator/1');
    add_line(alg, 'Kernel/10', 'Gmem0WriteCreator/2');
    add_line(alg, 'Kernel/11', 'Gmem0WriteCreator/3');
    add_line(alg, 'Kernel/12', 'gmem0_wrData/1');
    add_line(alg, 'Gmem0WriteCreator/1', 'gmem0_wrCtrlOut/1');

    add_line(alg, 'Kernel/13', 'done/1');
    add_line(alg, 'Kernel/14', 'max_delta/1');
end

function connect_top_level(model_name)
    ins = {'gmem2_rdData','gmem2_rdCtrlIn','gmem1_rdData','gmem1_rdCtrlIn', ...
           'gmem0_wrCtrlIn','start','map_x','map_y','cu_id'};
    outs = {'gmem2_rdCtrlOut','gmem1_rdCtrlOut','gmem0_wrData','gmem0_wrCtrlOut', ...
            'done','max_delta'};

    for i = 1:numel(ins)
        add_line(model_name, [ins{i} '/1'], ['Algorithm/' num2str(i)]);
    end
    for i = 1:numel(outs)
        add_line(model_name, ['Algorithm/' num2str(i)], [outs{i} '/1']);
    end
end

function configure_hdl(model_name)
    dut_name = [model_name '/Algorithm'];

    hdlset_param(model_name, 'HDLSubsystem', [model_name '/Algorithm']);
    hdlset_param(model_name, 'Workflow', 'IP Core Generation');
    hdlset_param(model_name, 'SynthesisTool', 'Xilinx Vivado');
    hdlset_param(model_name, 'SynthesisToolChipFamily', 'Zynq UltraScale+');
    hdlset_param(model_name, 'SynthesisToolDeviceName', 'xczu3eg');
    hdlset_param(model_name, 'SynthesisToolPackageName', 'sbva484');
    hdlset_param(model_name, 'SynthesisToolSpeedValue', '-1');
    hdlset_param(model_name, 'UseFloatingPoint', 'off');

    set_named_mem_if(model_name, dut_name, 'gmem2_rdData', ...
                     'gmem2/Input Read Memory Channel', 'gmem2 Read', ...
                     'rdData', 'Data');
    set_named_mem_if(model_name, dut_name, 'gmem2_rdCtrlIn', ...
                     'gmem2/Input Read Memory Channel', 'gmem2 Read', ...
                     'rdCtrlOut', 'Read Slave to Master Bus');
    set_named_mem_if(model_name, dut_name, 'gmem2_rdCtrlOut', ...
                     'gmem2/Input Read Memory Channel', 'gmem2 Read', ...
                     'rdCtrlIn', 'Read Master to Slave Bus');

    set_named_mem_if(model_name, dut_name, 'gmem1_rdData', ...
                     'gmem1/Input Read Memory Channel', 'gmem1 Read', ...
                     'rdData', 'Data');
    set_named_mem_if(model_name, dut_name, 'gmem1_rdCtrlIn', ...
                     'gmem1/Input Read Memory Channel', 'gmem1 Read', ...
                     'rdCtrlOut', 'Read Slave to Master Bus');
    set_named_mem_if(model_name, dut_name, 'gmem1_rdCtrlOut', ...
                     'gmem1/Input Read Memory Channel', 'gmem1 Read', ...
                     'rdCtrlIn', 'Read Master to Slave Bus');

    set_named_mem_if(model_name, dut_name, 'gmem0_wrCtrlIn', ...
                     'gmem0/Output Write Channel', 'gmem0 Write', ...
                     'wrCtrlOut', 'Write Slave to Master Bus');
    set_named_mem_if(model_name, dut_name, 'gmem0_wrData', ...
                     'gmem0/Output Write Channel', 'gmem0 Write', ...
                     'wrData', 'Data');
    set_named_mem_if(model_name, dut_name, 'gmem0_wrCtrlOut', ...
                     'gmem0/Output Write Channel', 'gmem0 Write', ...
                     'wrCtrlIn', 'Write Master to Slave Bus');

    set_named_axi_reg(model_name, dut_name, 'start', 'axi4', 'AXI4-Lite', 'x"0100"');
    set_named_axi_reg(model_name, dut_name, 'map_x', 'axi4', 'AXI4-Lite', 'x"0110"');
    set_named_axi_reg(model_name, dut_name, 'map_y', 'axi4', 'AXI4-Lite', 'x"0120"');
    set_named_axi_reg(model_name, dut_name, 'cu_id', 'axi4', 'AXI4-Lite', 'x"0130"');
    set_named_axi_reg(model_name, dut_name, 'done', 'axi4', 'AXI4-Lite', 'x"0140"');
    set_named_axi_reg(model_name, dut_name, 'max_delta', 'axi4', 'AXI4-Lite', 'x"0150"');
end

function set_mem_if(block_path, interface_name, mapping)
    hdlset_param(block_path, 'IOInterface', interface_name);
    hdlset_param(block_path, 'IOInterfaceMapping', mapping);
end

function set_axi_reg(block_path, mapping)
    hdlset_param(block_path, 'IOInterface', 'axi4');
    hdlset_param(block_path, 'IOInterfaceMapping', mapping);
end

function set_named_mem_if(model_name, dut_name, block_name, model_interface, dut_interface, ...
                          model_mapping, dut_mapping)
    set_mem_if([model_name '/' block_name], model_interface, model_mapping);
    set_mem_if([dut_name '/' block_name], dut_interface, dut_mapping);
end

function set_named_axi_reg(model_name, dut_name, block_name, model_interface, dut_interface, mapping)
    hdlset_param([model_name '/' block_name], 'IOInterface', model_interface);
    hdlset_param([model_name '/' block_name], 'IOInterfaceMapping', mapping);
    hdlset_param([dut_name '/' block_name], 'IOInterface', dut_interface);
    hdlset_param([dut_name '/' block_name], 'IOInterfaceMapping', mapping);
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end
