# ===========================================================================
# build_vivado.tcl — Vivado synthesis, implementation, bitstream
# Usage: vivado -mode batch -source build_vivado.tcl -tclargs <tile|stream|matlab>
# ===========================================================================

if {$argc < 2} {
    error "Usage: vivado -mode batch -source build_vivado.tcl -tclargs <tile|stream|matlab> <build_dir>"
}
set variant   [lindex $argv 0]
set build_dir [file normalize [lindex $argv 1]]
if {$variant ni {tile stream matlab}} {
    error "Invalid variant '$variant'. Must be 'tile', 'stream', or 'matlab'."
}

set script_dir   [file normalize [file dirname [info script]]]
set project_name "vi_${variant}"
set xpr_file     "$build_dir/$project_name/$project_name.xpr"
set run_jobs 2

if {$variant eq "matlab"} {
    # The MATLAB-generated IP is large enough to exhaust memory when OOC
    # synthesis runs are launched concurrently on typical development hosts.
    set run_jobs 1
}

if {[info exists ::env(VI_VIVADO_JOBS)] && $::env(VI_VIVADO_JOBS) ne ""} {
    set run_jobs $::env(VI_VIVADO_JOBS)
}

proc ensure_run_is_launchable {run_name} {
    set run [get_runs $run_name -quiet]
    if {[llength $run] == 0} {
        return
    }

    set status [get_property STATUS $run]
    set needs_refresh [get_property NEEDS_REFRESH $run]
    if {$needs_refresh || ![string match "Not started*" $status]} {
        puts "INFO: Resetting $run_name (STATUS='$status', NEEDS_REFRESH=$needs_refresh)"
        reset_run $run_name
    }
}

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

# Reset runs so they can always be re-launched after HLS/IP regeneration
ensure_run_is_launchable synth_1

# Synthesis (incremental — skips unchanged OOC blocks)
puts "INFO: Launching synth_1 with -jobs $run_jobs"
launch_runs synth_1 -jobs $run_jobs
wait_on_run synth_1
if {[get_property STATUS [get_runs synth_1]] != "synth_design Complete!"} {
    error "Synthesis failed"
}

# Reset implementation run before re-launch
ensure_run_is_launchable impl_1

# Implementation + bitstream
puts "INFO: Launching impl_1 with -jobs $run_jobs"
launch_runs impl_1 -to_step write_bitstream -jobs $run_jobs
wait_on_run impl_1
if {[get_property STATUS [get_runs impl_1]] != "write_bitstream Complete!"} {
    error "Implementation/bitstream failed"
}

puts "INFO: Bitstream generated successfully for variant '$variant'"
