# ===========================================================================
# build_vivado.tcl — Vivado synthesis, implementation, bitstream
# Usage: vivado -mode batch -source build_vivado.tcl -tclargs <tile|stream>
# ===========================================================================

if {$argc < 2} {
    error "Usage: vivado -mode batch -source build_vivado.tcl -tclargs <tile|stream> <build_dir>"
}
set variant   [lindex $argv 0]
set build_dir [file normalize [lindex $argv 1]]
if {$variant ni {tile stream}} {
    error "Invalid variant '$variant'. Must be 'tile' or 'stream'."
}

set script_dir   [file normalize [file dirname [info script]]]
set project_name "vi_${variant}"
set xpr_file     "$build_dir/$project_name/$project_name.xpr"

if {![file exists $xpr_file]} {
    puts "INFO: Project not found, creating..."
    set ::build_dir $build_dir
    source "$script_dir/create_project_${variant}.tcl"
} else {
    open_project $xpr_file
}

# Upgrade any locked IPs (e.g., after HLS IP regeneration)
set locked [get_ips -filter {IS_LOCKED == 1} -quiet]
if {[llength $locked] > 0} {
    puts "INFO: Upgrading locked IPs: $locked"
    upgrade_ip $locked
}

# Reset stale runs so they can be re-launched
if {[get_property NEEDS_REFRESH [get_runs synth_1]]} {
    puts "INFO: Resetting synth_1 (stale)"
    reset_run synth_1
}

# Synthesis (incremental — skips unchanged OOC blocks)
launch_runs synth_1 -jobs 6
wait_on_run synth_1
if {[get_property STATUS [get_runs synth_1]] != "synth_design Complete!"} {
    error "Synthesis failed"
}

# Reset impl run if stale
if {[get_property NEEDS_REFRESH [get_runs impl_1]]} {
    puts "INFO: Resetting impl_1 (stale)"
    reset_run impl_1
}

# Implementation + bitstream
launch_runs impl_1 -to_step write_bitstream -jobs 6
wait_on_run impl_1
if {[get_property STATUS [get_runs impl_1]] != "write_bitstream Complete!"} {
    error "Implementation/bitstream failed"
}

puts "INFO: Bitstream generated successfully for variant '$variant'"
