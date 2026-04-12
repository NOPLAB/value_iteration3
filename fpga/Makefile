FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
SCRIPTS_DIR := $(FPGA_DIR)scripts/
VIVADO_DIR  := $(FPGA_DIR)vivado/ultra96v2/

.PHONY: csim_tile hls_tile vivado_tile bitstream_tile \
        csim_stream hls_stream vivado_stream bitstream_stream \
        clean_hls_tile clean_hls_stream clean_vivado_tile clean_vivado_stream clean

# ---------- Tile-based kernel ----------

csim_tile:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl run_csim_tile.tcl

hls_tile:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl export_hls_ip_tile.tcl
	cd $(VIVADO_DIR)ip_repo_tile && unzip -o export.zip

vivado_tile: hls_tile
	vivado -mode batch -source $(SCRIPTS_DIR)build_vivado.tcl -tclargs tile

bitstream_tile: vivado_tile

# ---------- Streaming kernel ----------

csim_stream:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl run_csim_stream.tcl

hls_stream:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl export_hls_ip_stream.tcl
	cd $(VIVADO_DIR)ip_repo_stream && unzip -o export.zip

vivado_stream: hls_stream
	vivado -mode batch -source $(SCRIPTS_DIR)build_vivado.tcl -tclargs stream

bitstream_stream: vivado_stream

# ---------- Clean ----------

clean_hls_tile:
	if exist "$(subst /,\,$(SCRIPTS_DIR)hls_build_tile)" rmdir /s /q "$(subst /,\,$(SCRIPTS_DIR)hls_build_tile)"

clean_hls_stream:
	if exist "$(subst /,\,$(SCRIPTS_DIR)hls_build_stream)" rmdir /s /q "$(subst /,\,$(SCRIPTS_DIR)hls_build_stream)"

clean_vivado_tile:
	if exist "$(subst /,\,$(VIVADO_DIR)vi_tile)" rmdir /s /q "$(subst /,\,$(VIVADO_DIR)vi_tile)"

clean_vivado_stream:
	if exist "$(subst /,\,$(VIVADO_DIR)vi_stream)" rmdir /s /q "$(subst /,\,$(VIVADO_DIR)vi_stream)"

clean: clean_hls_tile clean_hls_stream clean_vivado_tile clean_vivado_stream
