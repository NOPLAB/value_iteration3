FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
TCL_DIR     := $(FPGA_DIR)tcl/
VIVADO_DIR  := $(FPGA_DIR)vivado/ultra96v2/
BUILD_DIR   := $(FPGA_DIR)build/

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
# Use order-only prerequisite to create build dir once.

csim: | $(BUILD_DIR)
	cd $(BUILD_DIR) && vitis-run --mode hls --tcl $(abspath $(TCL_DIR)run_csim_$(KERNEL).tcl)

hls: | $(BUILD_DIR)
	cd $(BUILD_DIR) && vitis-run --mode hls --tcl $(abspath $(TCL_DIR)export_hls_ip_$(KERNEL).tcl)
	cd $(VIVADO_DIR)ip_repo_$(KERNEL) && unzip -o export.zip

vivado: hls | $(BUILD_DIR)
	vivado -mode batch -source $(TCL_DIR)build_vivado.tcl -tclargs $(KERNEL) $(abspath $(BUILD_DIR))

bitstream: vivado

$(BUILD_DIR):
	-mkdir "$(subst /,\,$(BUILD_DIR))"

# ---------- Clean ----------

clean_hls:
	if exist "$(subst /,\,$(BUILD_DIR)hls_build_$(KERNEL))" rmdir /s /q "$(subst /,\,$(BUILD_DIR)hls_build_$(KERNEL))"

clean_vivado:
	if exist "$(subst /,\,$(BUILD_DIR)vi_$(KERNEL))" rmdir /s /q "$(subst /,\,$(BUILD_DIR)vi_$(KERNEL))"

ifdef KERNEL
clean: clean_hls clean_vivado
else
clean:
	if exist "$(subst /,\,$(BUILD_DIR))" rmdir /s /q "$(subst /,\,$(BUILD_DIR))"
endif
