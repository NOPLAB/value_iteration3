SCRIPTS_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

.PHONY: hls vivado bitstream csim clean_hls clean_vivado clean all

all: bitstream

# C-simulation (testbench)
csim:
	cd $(SCRIPTS_DIR) && vitis_hls -f run_csim.tcl

# HLS synthesis + IP export
hls:
	cd $(SCRIPTS_DIR) && vitis_hls -f export_hls_ip.tcl

# Vivado project creation + synthesis + bitstream
vivado: hls
	vivado -mode batch -source $(SCRIPTS_DIR)/build_vivado.tcl

bitstream: vivado

clean_hls:
	rm -rf $(SCRIPTS_DIR)/hls_build

clean_vivado:
	rm -rf $(SCRIPTS_DIR)/../vivado/ultra96v2/vi_ultra96v2

clean: clean_hls clean_vivado
