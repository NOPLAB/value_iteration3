# ===========================================================================
# export_hls_ip.tcl — Run Vitis HLS synthesis and export IP
# Usage: vitis_hls -f export_hls_ip.tcl
# ===========================================================================

set script_dir [file normalize [file dirname [info script]]]
set hls_dir    [file normalize "$script_dir/../hls/tile"]
set ip_dst     [file normalize "$script_dir/../vivado/ultra96v2/ip_repo_tile"]
set part       "xczu3eg-sbva484-1-i"

if {[file exists hls_build_tile/hls_build_tile.aps]} {
    open_project hls_build_tile
} else {
    open_project -reset hls_build_tile
}
set_top vi_sweep
add_files "$hls_dir/src/vi_sweep_top.cpp"
add_files "$hls_dir/src/compute_bellman.cpp"
add_files "$hls_dir/src/load_tiles.cpp"
add_files "$hls_dir/src/store_tiles.cpp"
add_files -tb "$hls_dir/tb/vi_sweep_tb.cpp"
add_files -tb "$hls_dir/tb/vi_reference.cpp"

if {[file exists hls_build_tile/solution1/solution1.aps]} {
    open_solution "solution1"
} else {
    open_solution -reset "solution1" -flow_target vivado
}
set_part $part
create_clock -period 6.67 -name default

# Synthesize
csynth_design

# Export IP
export_design -format ip_catalog -output $ip_dst

close_project
puts "INFO: HLS IP (vi_sweep) exported to $ip_dst"
