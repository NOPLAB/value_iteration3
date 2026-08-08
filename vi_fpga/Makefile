FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
TCL_DIR     := $(FPGA_DIR)tcl/
VIVADO_DIR  := $(FPGA_DIR)vivado/ultra96v2/
BUILD_DIR   := $(FPGA_DIR)build/

# ---------- Kernel selector ----------
# Usage: make -C vi_fpga csim KERNEL=stream
#        make -C vi_fpga bitstream KERNEL=tile
KERNEL ?= tile

.PHONY: csim hls vivado bitstream clean_hls clean_vivado clean

# ---------- HLS source dependencies ----------

HLS_COMMON     := $(wildcard $(FPGA_DIR)hls/common/*.cpp) \
                  $(wildcard $(FPGA_DIR)hls/common/*.h)

HLS_STREAM_SRC := $(wildcard $(FPGA_DIR)hls/stream/src/*.cpp) \
                  $(wildcard $(FPGA_DIR)hls/stream/src/*.h)
HLS_STREAM_TB  := $(wildcard $(FPGA_DIR)hls/stream/tb/*.cpp) $(HLS_COMMON)

HLS_TILE_SRC   := $(wildcard $(FPGA_DIR)hls/tile/src/*.cpp) \
                  $(wildcard $(FPGA_DIR)hls/tile/src/*.h)
HLS_TILE_TB    := $(wildcard $(FPGA_DIR)hls/tile/tb/*.cpp) $(HLS_COMMON)

HLS_CSIM_STAMP  := $(BUILD_DIR).csim_$(KERNEL)_done
HLS_SYNTH_STAMP := $(BUILD_DIR).hls_$(KERNEL)_done

# ---------- Build targets ----------

csim: $(HLS_CSIM_STAMP)

$(BUILD_DIR).csim_stream_done: $(HLS_STREAM_SRC) $(HLS_STREAM_TB) | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)run_csim_stream.tcl"
	@touch $@

$(BUILD_DIR).csim_tile_done: $(HLS_TILE_SRC) $(HLS_TILE_TB) | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)run_csim_tile.tcl"
	@touch $@

hls: $(HLS_SYNTH_STAMP)

$(BUILD_DIR).hls_stream_done: $(HLS_STREAM_SRC) | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)export_hls_ip_stream.tcl"
	unzip -o "$(VIVADO_DIR)ip_repo_stream.zip" -d "$(VIVADO_DIR)ip_repo_stream"
	@touch $@

$(BUILD_DIR).hls_tile_done: $(HLS_TILE_SRC) | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)export_hls_ip_tile.tcl"
	unzip -o "$(VIVADO_DIR)ip_repo_tile.zip" -d "$(VIVADO_DIR)ip_repo_tile"
	@touch $@

vivado: hls | $(BUILD_DIR)
	vivado -mode batch -source "$(TCL_DIR)build_vivado.tcl" -tclargs $(KERNEL) "$(BUILD_DIR)"

bitstream: vivado

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---------- Clean ----------

clean_hls:
	rm -rf $(BUILD_DIR)hls_build_$(KERNEL)
	rm -f "$(BUILD_DIR).csim_$(KERNEL)_done" "$(BUILD_DIR).hls_$(KERNEL)_done"

clean_vivado:
	rm -rf $(BUILD_DIR)vi_$(KERNEL)

clean:
	rm -rf $(BUILD_DIR)
