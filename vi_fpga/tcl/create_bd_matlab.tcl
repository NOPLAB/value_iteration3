# ===========================================================================
# create_bd_matlab.tcl — Block Design: Zynq PS + 1x MATLAB HDL Coder IP
# ===========================================================================

create_bd_design "vi_bd"

set zynq [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
    -config {apply_board_preset "1"} $zynq

set_property -dict [list \
    CONFIG.PSU__USE__CLK0 {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__ACT_FREQMHZ {100.000000} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__DIVISOR0 {15} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__DIVISOR1 {1} \
    CONFIG.PSU__USE__S_AXI_GP2 {1} \
    CONFIG.PSU__SAXIGP2__DATA_WIDTH {128} \
    CONFIG.PSU__USE__S_AXI_GP3 {1} \
    CONFIG.PSU__SAXIGP3__DATA_WIDTH {128} \
    CONFIG.PSU__USE__S_AXI_GP4 {1} \
    CONFIG.PSU__SAXIGP4__DATA_WIDTH {128} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
] $zynq

set cu0 [create_bd_cell -type ip -vlnv xilinx.com:ip:Algorithm_ip:1.0 vi_matlab_cu0]

set ctrl_smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 ctrl_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $ctrl_smc

set gmem0_smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 gmem0_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $gmem0_smc

set gmem1_smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 gmem1_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $gmem1_smc

set gmem2_smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 gmem2_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $gmem2_smc

set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0]

set clk [get_bd_pins zynq_ps/pl_clk0]
set rstn [get_bd_pins proc_sys_reset_0/peripheral_aresetn]

connect_bd_net $clk \
    [get_bd_pins ctrl_smc/aclk] \
    [get_bd_pins gmem0_smc/aclk] \
    [get_bd_pins gmem1_smc/aclk] \
    [get_bd_pins gmem2_smc/aclk] \
    [get_bd_pins vi_matlab_cu0/AXI4_Lite_ACLK] \
    [get_bd_pins vi_matlab_cu0/IPCORE_CLK] \
    [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
    [get_bd_pins zynq_ps/saxihp0_fpd_aclk] \
    [get_bd_pins zynq_ps/saxihp1_fpd_aclk] \
    [get_bd_pins zynq_ps/saxihp2_fpd_aclk] \
    [get_bd_pins zynq_ps/maxihpm0_fpd_aclk]

connect_bd_net [get_bd_pins zynq_ps/pl_resetn0] [get_bd_pins proc_sys_reset_0/ext_reset_in]

connect_bd_net $rstn \
    [get_bd_pins ctrl_smc/aresetn] \
    [get_bd_pins gmem0_smc/aresetn] \
    [get_bd_pins gmem1_smc/aresetn] \
    [get_bd_pins gmem2_smc/aresetn] \
    [get_bd_pins vi_matlab_cu0/AXI4_Lite_ARESETN] \
    [get_bd_pins vi_matlab_cu0/IPCORE_RESETN]

connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_HPM0_FPD] [get_bd_intf_pins ctrl_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins ctrl_smc/M00_AXI] [get_bd_intf_pins vi_matlab_cu0/AXI4_Lite]

connect_bd_intf_net [get_bd_intf_pins vi_matlab_cu0/gmem0] [get_bd_intf_pins gmem0_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins gmem0_smc/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP0_FPD]

connect_bd_intf_net [get_bd_intf_pins vi_matlab_cu0/gmem1] [get_bd_intf_pins gmem1_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins gmem1_smc/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP1_FPD]

connect_bd_intf_net [get_bd_intf_pins vi_matlab_cu0/gmem2] [get_bd_intf_pins gmem2_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins gmem2_smc/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP2_FPD]

assign_bd_address [get_bd_addr_segs zynq_ps/SAXIGP2/HP0_DDR_LOW]
assign_bd_address [get_bd_addr_segs zynq_ps/SAXIGP3/HP1_DDR_LOW]
assign_bd_address [get_bd_addr_segs zynq_ps/SAXIGP4/HP2_DDR_LOW]
assign_bd_address [get_bd_addr_segs vi_matlab_cu0/AXI4_Lite/AXI4_Lite]

validate_bd_design
save_bd_design

puts "INFO: Block design 'vi_bd' created (1 CU MATLAB HDL Coder IP)"
