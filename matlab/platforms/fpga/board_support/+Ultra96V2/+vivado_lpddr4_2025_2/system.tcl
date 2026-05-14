namespace eval _tcl {
proc get_script_folder {} {
   set script_path [file normalize [info script]]
   set script_folder [file dirname $script_path]
   return $script_folder
}
}

variable script_folder
set script_folder [_tcl::get_script_folder]

set board_part "Avnet-tria:Ultra96v2:part0:1.3"
set fpga_part "xczu3eg-sbva484-1-i"
set design_name "system"

proc ensure_project {board_part fpga_part} {
    if {[get_projects -quiet] eq ""} {
        set project_root [file normalize [file join $::script_folder .. .. .. build board_support_project]]
        create_project project_1 $project_root -part $fpga_part -force
    }

    if {[llength [get_board_parts -quiet $board_part]] > 0} {
        set_property BOARD_PART $board_part [current_project]
    } else {
        puts "WARNING: Vivado board part $board_part was not found. Using FPGA part only."
    }
}

proc ensure_design {design_name} {
    set cur_design [current_bd_design -quiet]
    set list_cells [get_bd_cells -quiet]

    if {$cur_design ne "" && $list_cells eq ""} {
        if {$cur_design ne $design_name} {
            current_bd_design $cur_design
        }
        return
    }

    if {$cur_design eq $design_name && $list_cells ne ""} {
        error "Design <$design_name> already exists in the current project."
    }

    if {[get_files -quiet ${design_name}.bd] ne ""} {
        error "Design <$design_name> already exists in the current project."
    }

    create_bd_design $design_name
    current_bd_design $design_name
}

proc create_root_design {} {
    set zynq_ultra_ps_e_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ultra_ps_e_0]

    if {[llength [get_board_parts -quiet "Avnet-tria:Ultra96v2:part0:1.3"]] > 0} {
        apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
            -config {apply_board_preset "1"} $zynq_ultra_ps_e_0
    }

    set_property -dict [list \
        CONFIG.PSU__USE__CLK0 {1} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__ACT_FREQMHZ {100.000000} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__DIVISOR0 {15} \
        CONFIG.PSU__CRL_APB__PL0_REF_CTRL__DIVISOR1 {1} \
        CONFIG.PSU__USE__M_AXI_GP0 {1} \
        CONFIG.PSU__MAXIGP0__DATA_WIDTH {32} \
        CONFIG.PSU__USE__M_AXI_GP1 {0} \
        CONFIG.PSU__USE__S_AXI_GP2 {1} \
        CONFIG.PSU__SAXIGP2__DATA_WIDTH {128} \
        CONFIG.PSU__USE__S_AXI_GP3 {1} \
        CONFIG.PSU__SAXIGP3__DATA_WIDTH {128} \
        CONFIG.PSU__USE__S_AXI_GP4 {1} \
        CONFIG.PSU__SAXIGP4__DATA_WIDTH {128} \
    ] $zynq_ultra_ps_e_0

    set axi_ctrl_interconnect [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_ctrl_interconnect]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $axi_ctrl_interconnect

    set axi_hp0_interconnect [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_hp0_interconnect]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $axi_hp0_interconnect

    set axi_hp1_interconnect [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_hp1_interconnect]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $axi_hp1_interconnect

    set axi_hp2_interconnect [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_hp2_interconnect]
    set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $axi_hp2_interconnect

    set proc_sys_reset_0 [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0]

    connect_bd_intf_net [get_bd_intf_pins zynq_ultra_ps_e_0/M_AXI_HPM0_FPD] \
        [get_bd_intf_pins axi_ctrl_interconnect/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins axi_hp0_interconnect/M00_AXI] \
        [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HP0_FPD]
    connect_bd_intf_net [get_bd_intf_pins axi_hp1_interconnect/M00_AXI] \
        [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HP1_FPD]
    connect_bd_intf_net [get_bd_intf_pins axi_hp2_interconnect/M00_AXI] \
        [get_bd_intf_pins zynq_ultra_ps_e_0/S_AXI_HP2_FPD]

    connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_resetn0] \
        [get_bd_pins proc_sys_reset_0/ext_reset_in]

    connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] \
        [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
        [get_bd_pins zynq_ultra_ps_e_0/maxihpm0_fpd_aclk] \
        [get_bd_pins zynq_ultra_ps_e_0/maxihpm0_lpd_aclk] \
        [get_bd_pins zynq_ultra_ps_e_0/saxihp0_fpd_aclk] \
        [get_bd_pins zynq_ultra_ps_e_0/saxihp1_fpd_aclk] \
        [get_bd_pins zynq_ultra_ps_e_0/saxihp2_fpd_aclk] \
        [get_bd_pins axi_ctrl_interconnect/ACLK] \
        [get_bd_pins axi_ctrl_interconnect/S00_ACLK] \
        [get_bd_pins axi_ctrl_interconnect/M00_ACLK] \
        [get_bd_pins axi_hp0_interconnect/ACLK] \
        [get_bd_pins axi_hp0_interconnect/S00_ACLK] \
        [get_bd_pins axi_hp0_interconnect/M00_ACLK] \
        [get_bd_pins axi_hp1_interconnect/ACLK] \
        [get_bd_pins axi_hp1_interconnect/S00_ACLK] \
        [get_bd_pins axi_hp1_interconnect/M00_ACLK] \
        [get_bd_pins axi_hp2_interconnect/ACLK] \
        [get_bd_pins axi_hp2_interconnect/S00_ACLK] \
        [get_bd_pins axi_hp2_interconnect/M00_ACLK]

    connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
        [get_bd_pins axi_ctrl_interconnect/ARESETN] \
        [get_bd_pins axi_ctrl_interconnect/S00_ARESETN] \
        [get_bd_pins axi_ctrl_interconnect/M00_ARESETN] \
        [get_bd_pins axi_hp0_interconnect/ARESETN] \
        [get_bd_pins axi_hp0_interconnect/S00_ARESETN] \
        [get_bd_pins axi_hp0_interconnect/M00_ARESETN] \
        [get_bd_pins axi_hp1_interconnect/ARESETN] \
        [get_bd_pins axi_hp1_interconnect/S00_ARESETN] \
        [get_bd_pins axi_hp1_interconnect/M00_ARESETN] \
        [get_bd_pins axi_hp2_interconnect/ARESETN] \
        [get_bd_pins axi_hp2_interconnect/S00_ARESETN] \
        [get_bd_pins axi_hp2_interconnect/M00_ARESETN]

    assign_bd_address [get_bd_addr_segs zynq_ultra_ps_e_0/SAXIGP2/HP0_DDR_LOW]
    assign_bd_address [get_bd_addr_segs zynq_ultra_ps_e_0/SAXIGP3/HP1_DDR_LOW]
    assign_bd_address [get_bd_addr_segs zynq_ultra_ps_e_0/SAXIGP4/HP2_DDR_LOW]

    validate_bd_design
    save_bd_design
}

ensure_project $board_part $fpga_part
ensure_design $design_name
create_root_design
