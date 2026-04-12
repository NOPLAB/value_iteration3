FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
SCRIPTS_DIR := $(FPGA_DIR)scripts/

.PHONY: csim hls vivado bitstream csim_stream hls_stream \
        clean_hls clean_vivado clean_hls_stream clean all

all: bitstream

# ---------- Tile-based kernel (legacy) ----------

csim:
	cd $(SCRIPTS_DIR) && vitis_hls -f run_csim_tile.tcl

hls:
	cd $(SCRIPTS_DIR) && vitis_hls -f export_hls_ip_tile.tcl

# ---------- Streaming kernel ----------

csim_stream:
	cd $(SCRIPTS_DIR) && vitis_hls -f run_csim_stream.tcl

hls_stream:
	cd $(SCRIPTS_DIR) && vitis_hls -f export_hls_ip_stream.tcl

# ---------- Vivado ----------

vivado: hls
	vivado -mode batch -source $(SCRIPTS_DIR)build_vivado.tcl

bitstream: vivado

# ---------- Clean ----------

clean_hls:
	rm -rf $(SCRIPTS_DIR)hls_build_tile

clean_hls_stream:
	rm -rf $(SCRIPTS_DIR)hls_build_stream

clean_vivado:
	rm -rf $(FPGA_DIR)vivado/ultra96v2/vi_ultra96v2

clean: clean_hls clean_hls_stream clean_vivado
