# ===========================================================================
# build_vivado.tcl — Vivado synthesis, implementation, bitstream
# Usage: vivado -mode batch -source build_vivado.tcl
# ===========================================================================

set script_dir   [file normalize [file dirname [info script]]]
set project_dir  [file normalize "$script_dir/../vivado/ultra96v2"]
set project_name "vi_ultra96v2"
set xpr_file     "$project_dir/$project_name/$project_name.xpr"

if {![file exists $xpr_file]} {
    puts "INFO: Project not found, creating..."
    source "$project_dir/create_project.tcl"
} else {
    open_project $xpr_file
}

# Synthesis
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property STATUS [get_runs synth_1]] != "synth_design Complete!"} {
    error "Synthesis failed"
}

# Implementation + bitstream
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property STATUS [get_runs impl_1]] != "write_bitstream Complete!"} {
    error "Implementation/bitstream failed"
}

puts "INFO: Bitstream generated successfully"
