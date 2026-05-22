FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
TCL_DIR     := $(FPGA_DIR)tcl/
VIVADO_DIR  := $(FPGA_DIR)vivado/ultra96v2/
BUILD_DIR   := $(FPGA_DIR)build/

# ---------- OS detection ----------
ifeq ($(OS),Windows_NT)
  MKDIR = if not exist "$(subst /,\,$1)" mkdir "$(subst /,\,$1)"
  RMDIR = if exist "$(subst /,\,$1)" rmdir /s /q "$(subst /,\,$1)"
  TOUCH = powershell -NoProfile -Command "$$path='$(subst /,\,$1)'; if (Test-Path $$path) { $$item=Get-Item $$path; $$item.LastWriteTime=Get-Date } else { New-Item -ItemType File -Path $$path | Out-Null }"
else
  MKDIR = mkdir -p $1
  RMDIR = rm -rf $1
  TOUCH = touch $1
endif

# ---------- Kernel selector ----------
# Usage: make -C fpga csim stream
#        make -C fpga bitstream tile

ifneq ($(filter stream,$(MAKECMDGOALS)),)
  KERNEL := stream
endif
ifneq ($(filter tile,$(MAKECMDGOALS)),)
  KERNEL := tile
endif

stream tile:
	@:

.PHONY: stream tile csim hls vivado bitstream clean_hls clean_vivado clean

# ---------- HLS source dependencies ----------

HLS_STREAM_SRC := $(wildcard $(FPGA_DIR)hls/stream/src/*.cpp) \
                  $(wildcard $(FPGA_DIR)hls/stream/src/*.h)
HLS_STREAM_TB  := $(wildcard $(FPGA_DIR)hls/stream/tb/*.cpp)

HLS_TILE_SRC   := $(wildcard $(FPGA_DIR)hls/tile/src/*.cpp) \
                  $(wildcard $(FPGA_DIR)hls/tile/src/*.h)
HLS_TILE_TB    := $(wildcard $(FPGA_DIR)hls/tile/tb/*.cpp)

HLS_CSIM_STAMP  := $(BUILD_DIR).csim_$(KERNEL)_done
HLS_SYNTH_STAMP := $(BUILD_DIR).hls_$(KERNEL)_done

# ---------- Build targets ----------

csim: $(HLS_CSIM_STAMP)

$(BUILD_DIR).csim_stream_done: $(HLS_STREAM_SRC) $(HLS_STREAM_TB) | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)run_csim_stream.tcl"
	@$(call TOUCH,$@)

$(BUILD_DIR).csim_tile_done: $(HLS_TILE_SRC) $(HLS_TILE_TB) | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)run_csim_tile.tcl"
	@$(call TOUCH,$@)

hls: $(HLS_SYNTH_STAMP)

$(BUILD_DIR).hls_stream_done: $(HLS_STREAM_SRC) | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)export_hls_ip_stream.tcl"
	unzip -o "$(VIVADO_DIR)ip_repo_stream.zip" -d "$(VIVADO_DIR)ip_repo_stream"
	@$(call TOUCH,$@)

$(BUILD_DIR).hls_tile_done: $(HLS_TILE_SRC) | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)export_hls_ip_tile.tcl"
	unzip -o "$(VIVADO_DIR)ip_repo_tile.zip" -d "$(VIVADO_DIR)ip_repo_tile"
	@$(call TOUCH,$@)

vivado: hls | $(BUILD_DIR)
	vivado -mode batch -source "$(TCL_DIR)build_vivado.tcl" -tclargs $(KERNEL) "$(BUILD_DIR)"

bitstream: vivado

$(BUILD_DIR):
	-$(call MKDIR,$(BUILD_DIR))

# ---------- Clean ----------

clean_hls:
	$(call RMDIR,$(BUILD_DIR)hls_build_$(KERNEL))
ifeq ($(OS),Windows_NT)
	-del "$(subst /,\,$(BUILD_DIR)).csim_$(KERNEL)_done" 2>NUL
	-del "$(subst /,\,$(BUILD_DIR)).hls_$(KERNEL)_done" 2>NUL
else
	-rm -f "$(BUILD_DIR).csim_$(KERNEL)_done" "$(BUILD_DIR).hls_$(KERNEL)_done"
endif

clean_vivado:
	$(call RMDIR,$(BUILD_DIR)vi_$(KERNEL))

ifdef KERNEL
clean: clean_hls clean_vivado
else
clean:
	$(call RMDIR,$(BUILD_DIR))
endif
