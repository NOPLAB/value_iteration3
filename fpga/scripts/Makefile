SCRIPTS_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

.PHONY: hls vivado bitstream csim clean_hls clean_vivado clean all csim_stream hls_stream clean_hls_stream

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

# Line-buffer streaming kernel
csim_stream:
	cd $(SCRIPTS_DIR) && vitis_hls -f run_csim_stream.tcl

hls_stream:
	cd $(SCRIPTS_DIR) && vitis_hls -f export_hls_ip_stream.tcl

clean_hls_stream:
	rm -rf $(SCRIPTS_DIR)/hls_build_stream
