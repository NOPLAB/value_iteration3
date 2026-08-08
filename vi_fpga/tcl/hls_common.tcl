# ===========================================================================
# hls_common.tcl — Shared Vitis HLS driver for both kernels.
# Callers set before sourcing:
#   kernel : tile | stream
#   mode   : csim | export
# ===========================================================================

set script_dir [file normalize [file dirname [info script]]]
set hls_dir    [file normalize "$script_dir/../hls/$kernel"]
set common_dir [file normalize "$script_dir/../hls/common"]
# vi_reference.cpp includes the shared C implementations from these dirs.
set driver_dir [file normalize "$script_dir/../driver/uio"]
set hostsrc_dir [file normalize "$script_dir/../host/src"]
set tb_cflags  "-I$common_dir -I$driver_dir -I$hostsrc_dir"
set part       "xczu3eg-sbva484-1-i"
set top        [expr {$kernel eq "stream" ? "vi_sweep_stream" : "vi_sweep"}]
set proj       hls_build_$kernel

if {[file exists $proj/$proj.aps]} {
    open_project $proj
} else {
    open_project -reset $proj
}
set_top $top
foreach f [lsort [glob "$hls_dir/src/*.cpp"]] {
    add_files $f
}
foreach f [lsort [glob "$hls_dir/tb/*.cpp"]] {
    add_files -tb $f -cflags $tb_cflags
}
add_files -tb "$common_dir/vi_reference.cpp" -cflags $tb_cflags

if {[file exists $proj/solution1/solution1.aps]} {
    open_solution "solution1"
} else {
    open_solution -reset "solution1" -flow_target vivado
}
set_part $part
create_clock -period 6.67 -name default

if {$mode eq "csim"} {
    csim_design
} else {
    csynth_design
    set ip_dst [file normalize "$script_dir/../vivado/ultra96v2/ip_repo_$kernel"]
    export_design -format ip_catalog -output $ip_dst
    puts "INFO: HLS IP ($top) exported to $ip_dst"
}

close_project
