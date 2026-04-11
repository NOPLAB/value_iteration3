# ===========================================================================
# create_project.tcl — Ultra96-V2 Vivado project with vi_sweep IP
# ===========================================================================

set project_name "vi_ultra96v2"
set project_dir  [file normalize [file dirname [info script]]]
set ip_repo_dir  [file normalize "$project_dir/ip_repo"]
set part         "xczu3eg-sbva484-1-i"

create_project $project_name "$project_dir/$project_name" -part $part -force

set_property board_part Avnet-tria:Ultra96v2:part0:1.3 [current_project]

# Add HLS IP repo (contains vi_sweep IP)
set_property ip_repo_paths $ip_repo_dir [current_project]
update_ip_catalog

# Source block design
source "$project_dir/create_bd.tcl"

# Generate output products
generate_target all [get_files vi_bd.bd]

# Create HDL wrapper
make_wrapper -files [get_files vi_bd.bd] -top
add_files -norecurse [glob "$project_dir/$project_name/$project_name.gen/sources_1/bd/vi_bd/hdl/vi_bd_wrapper.v"]
update_compile_order -fileset sources_1

puts "INFO: Project created at $project_dir/$project_name"
