# ===========================================================================
# create_bd.tcl — Block Design: Zynq PS + 2x vi_sweep_stream HLS IP
# STRIP_W_MAX=145 → 63 BRAM36 per CU → 2 CUs fit in ZU3EG (138/216).
# ===========================================================================

create_bd_design "vi_bd"

# --- Zynq UltraScale+ PS ---
set zynq [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
    -config {apply_board_preset "1"} $zynq

# Enable HP0 + HP1 for data, disable unused HPM1
set_property -dict [list \
    CONFIG.PSU__USE__CLK0 {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__ACT_FREQMHZ {150.000000} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {150} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__DIVISOR0 {10} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__DIVISOR1 {1} \
    CONFIG.PSU__USE__S_AXI_GP2 {1} \
    CONFIG.PSU__SAXIGP2__DATA_WIDTH {128} \
    CONFIG.PSU__USE__S_AXI_GP3 {1} \
    CONFIG.PSU__SAXIGP3__DATA_WIDTH {128} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
] $zynq

# --- 2x vi_sweep_stream HLS IPs ---
set cu0 [create_bd_cell -type ip -vlnv xilinx.com:hls:vi_sweep_stream:1.0 vi_sweep_stream_cu0]
set cu1 [create_bd_cell -type ip -vlnv xilinx.com:hls:vi_sweep_stream:1.0 vi_sweep_stream_cu1]

# --- Data SmartConnect for CU0 (2 AXI masters -> HP0) ---
set data_smc0 [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 data_smc0]
set_property CONFIG.NUM_SI {2} $data_smc0

# --- Data SmartConnect for CU1 (2 AXI masters -> HP1) ---
set data_smc1 [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 data_smc1]
set_property CONFIG.NUM_SI {2} $data_smc1

# --- Control SmartConnect (1 GP master -> 2 control slaves) ---
set ctrl_smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 ctrl_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {2}] $ctrl_smc

# --- Reset ---
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0]

# --- Clock and reset wiring ---
set clk [get_bd_pins zynq_ps/pl_clk0]
set rstn [get_bd_pins proc_sys_reset_0/peripheral_aresetn]

connect_bd_net $clk \
    [get_bd_pins data_smc0/aclk] \
    [get_bd_pins data_smc1/aclk] \
    [get_bd_pins ctrl_smc/aclk] \
    [get_bd_pins vi_sweep_stream_cu0/ap_clk] \
    [get_bd_pins vi_sweep_stream_cu1/ap_clk] \
    [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
    [get_bd_pins zynq_ps/saxihp0_fpd_aclk] \
    [get_bd_pins zynq_ps/saxihp1_fpd_aclk] \
    [get_bd_pins zynq_ps/maxihpm0_fpd_aclk]

connect_bd_net [get_bd_pins zynq_ps/pl_resetn0] [get_bd_pins proc_sys_reset_0/ext_reset_in]

connect_bd_net $rstn \
    [get_bd_pins data_smc0/aresetn] \
    [get_bd_pins data_smc1/aresetn] \
    [get_bd_pins ctrl_smc/aresetn] \
    [get_bd_pins vi_sweep_stream_cu0/ap_rst_n] \
    [get_bd_pins vi_sweep_stream_cu1/ap_rst_n]

# --- Interrupt ---
set irq_concat [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 irq_concat]
set_property -dict [list CONFIG.NUM_PORTS {2} CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1}] $irq_concat
connect_bd_net [get_bd_pins vi_sweep_stream_cu0/interrupt] [get_bd_pins irq_concat/In0]
connect_bd_net [get_bd_pins vi_sweep_stream_cu1/interrupt] [get_bd_pins irq_concat/In1]
connect_bd_net [get_bd_pins irq_concat/dout] [get_bd_pins zynq_ps/pl_ps_irq0]
set_property -dict [list CONFIG.PSU__USE__IRQ0 {1} CONFIG.PSU__IRQ_P2F_IRQ0_SELECT {1}] [get_bd_cells zynq_ps]

# --- Control path ---
connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_HPM0_FPD] [get_bd_intf_pins ctrl_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins ctrl_smc/M00_AXI] [get_bd_intf_pins vi_sweep_stream_cu0/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins ctrl_smc/M01_AXI] [get_bd_intf_pins vi_sweep_stream_cu1/s_axi_control]

# --- Data path: CU0 -> HP0, CU1 -> HP1 ---
connect_bd_intf_net [get_bd_intf_pins vi_sweep_stream_cu0/m_axi_gmem0] [get_bd_intf_pins data_smc0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins vi_sweep_stream_cu0/m_axi_gmem1] [get_bd_intf_pins data_smc0/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins data_smc0/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP0_FPD]

connect_bd_intf_net [get_bd_intf_pins vi_sweep_stream_cu1/m_axi_gmem0] [get_bd_intf_pins data_smc1/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins vi_sweep_stream_cu1/m_axi_gmem1] [get_bd_intf_pins data_smc1/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins data_smc1/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP1_FPD]

# --- Address assignment ---
assign_bd_address [get_bd_addr_segs zynq_ps/SAXIGP2/HP0_DDR_LOW]
assign_bd_address [get_bd_addr_segs zynq_ps/SAXIGP3/HP1_DDR_LOW]
assign_bd_address [get_bd_addr_segs vi_sweep_stream_cu0/s_axi_control/Reg]
assign_bd_address [get_bd_addr_segs vi_sweep_stream_cu1/s_axi_control/Reg]

validate_bd_design
save_bd_design

puts "INFO: Block design 'vi_bd' created (2 CU streaming, HP0+HP1)"
