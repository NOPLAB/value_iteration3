# ===========================================================================
# create_project_matlab.tcl — Ultra96-V2 Vivado project (MATLAB HDL IP)
# ===========================================================================

set project_name "vi_matlab"
set tcl_dir      [file normalize [file dirname [info script]]]
set part         "xczu3eg-sbva484-1-i"
set ip_repo_dir  [file normalize "$::build_dir/matlab_ip_repo"]

if {![file exists "$ip_repo_dir/Algorithm_ip_v1_0/component.xml"]} {
    error "MATLAB IP repository not found at $ip_repo_dir. Run 'make matlab-hdl' first."
}

create_project $project_name "$::build_dir/$project_name" -part $part -force
set_property board_part Avnet-tria:Ultra96v2:part0:1.3 [current_project]

set_property ip_repo_paths $ip_repo_dir [current_project]
update_ip_catalog

source "$tcl_dir/create_bd_matlab.tcl"

generate_target all [get_files vi_bd.bd]
make_wrapper -files [get_files vi_bd.bd] -top
add_files -norecurse [glob "$::build_dir/$project_name/$project_name.gen/sources_1/bd/vi_bd/hdl/vi_bd_wrapper.v"]
update_compile_order -fileset sources_1

puts "INFO: MATLAB project created at $::build_dir/$project_name"
