FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
TCL_DIR     := $(FPGA_DIR)tcl/
VIVADO_DIR  := $(FPGA_DIR)vivado/ultra96v2/
BUILD_DIR   := $(FPGA_DIR)build/

# ---------- OS detection ----------
ifeq ($(OS),Windows_NT)
  MKDIR = if not exist "$(subst /,\,$1)" mkdir "$(subst /,\,$1)"
  RMDIR = if exist "$(subst /,\,$1)" rmdir /s /q "$(subst /,\,$1)"
else
  MKDIR = mkdir -p $1
  RMDIR = rm -rf $1
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

# ---------- Build targets ----------

csim: | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)run_csim_$(KERNEL).tcl"

hls: | $(BUILD_DIR)
	cd "$(BUILD_DIR)" && vitis-run --mode hls --tcl "$(TCL_DIR)export_hls_ip_$(KERNEL).tcl"
	unzip -o "$(VIVADO_DIR)ip_repo_$(KERNEL).zip" -d "$(VIVADO_DIR)ip_repo_$(KERNEL)"

vivado: hls | $(BUILD_DIR)
	vivado -mode batch -source "$(TCL_DIR)build_vivado.tcl" -tclargs $(KERNEL) "$(BUILD_DIR)"

bitstream: vivado

$(BUILD_DIR):
	-$(call MKDIR,$(BUILD_DIR))

# ---------- Clean ----------

clean_hls:
	$(call RMDIR,$(BUILD_DIR)hls_build_$(KERNEL))

clean_vivado:
	$(call RMDIR,$(BUILD_DIR)vi_$(KERNEL))

ifdef KERNEL
clean: clean_hls clean_vivado
else
clean:
	$(call RMDIR,$(BUILD_DIR))
endif

# ---------- MATLAB kernel targets ----------

.PHONY: matlab-sim matlab-hdl matlab-cosim matlab-bitstream

matlab-sim:
	cd "$(FPGA_DIR)matlab" && matlab -batch "addpath('src','testbench'); tb_full_sweep"

matlab-hdl:
	cd "$(FPGA_DIR)matlab" && matlab -batch "addpath('src','model'); cd model; create_model"

matlab-cosim:
	cd "$(FPGA_DIR)matlab" && matlab -batch "addpath('src','testbench'); cd cosim; cosim_tb"

matlab-bitstream:
	cd "$(FPGA_DIR)matlab" && matlab -batch "addpath('src','testbench','model'); cd soc; build_bitstream"
