# ===========================================================================
# export_hls_ip_stream.tcl — Run Vitis HLS synthesis and export IP
# Usage: vitis_hls -f export_hls_ip_stream.tcl
# ===========================================================================

set script_dir [file normalize [file dirname [info script]]]
set hls_dir    [file normalize "$script_dir/../hls/stream"]
set ip_dst     [file normalize "$script_dir/../vivado/ultra96v2/ip_repo_stream"]
set part       "xczu3eg-sbva484-1-i"

if {[file exists hls_build_stream/hls_build_stream.aps]} {
    open_project hls_build_stream
} else {
    open_project -reset hls_build_stream
}
set_top vi_sweep_stream
add_files "$hls_dir/src/vi_sweep_stream_top.cpp"
add_files "$hls_dir/src/stream_strip.cpp"
add_files "$hls_dir/src/compute_row.cpp"
add_files "$hls_dir/src/load_store_row.cpp"
add_files -tb "$hls_dir/tb/vi_sweep_stream_tb.cpp"
add_files -tb "$hls_dir/tb/vi_reference.cpp"

if {[file exists hls_build_stream/solution1/solution1.aps]} {
    open_solution "solution1"
} else {
    open_solution -reset "solution1" -flow_target vivado
}
set_part $part
create_clock -period 6.67 -name default

csynth_design
export_design -format ip_catalog -output $ip_dst

close_project
puts "INFO: HLS IP (vi_sweep_stream) exported to $ip_dst"
