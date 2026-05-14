function hB = plugin_board()
%PLUGIN_BOARD Board definition for the Avnet Ultra96-V2.

    hB = hdlcoder.Board;

    hB.BoardName = 'Avnet Ultra96-V2';

    hB.FPGAVendor = 'Xilinx';
    hB.FPGAFamily = 'Zynq UltraScale+';
    hB.FPGADevice = 'xczu3eg-sbva484-1-i';
    hB.FPGAPackage = '';
    hB.FPGASpeed = '';

    hB.SupportedTool = {'Xilinx Vivado'};
    hB.JTAGChainPosition = 1;

    % Keep the board definition minimal. The reference design owns the PS,
    % memory, and AXI topology used by HDL Workflow Advisor.
    hB.addExternalPortInterface( ...
        'IOPadConstraint', {'IOSTANDARD = LVCMOS18'});
end
