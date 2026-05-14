.PHONY: driver host test-host test-hw \
       csim hls vivado bitstream sync-hw-header \
       edf-docker edf-shell edf-setup edf-build \
       clean clean-fpga clean-edf

KERNEL ?=

# ---------- Software (driver + host) ----------

driver:
	$(MAKE) -C driver/uio all

host: driver
	$(MAKE) -C host all

test-host:
	$(MAKE) -C host test-host

test-hw:
	$(MAKE) -C host test-hw

# ---------- FPGA (HLS + Vivado) ----------
# Pass KERNEL= to select tile or stream, e.g.:
#   make csim KERNEL=stream
#   make bitstream KERNEL=tile

csim:
	$(MAKE) -C fpga csim $(KERNEL)

hls:
	$(MAKE) -C fpga hls $(KERNEL)

vivado:
	$(MAKE) -C fpga vivado $(KERNEL)

bitstream:
	$(MAKE) -C fpga bitstream $(KERNEL)

sync-hw-header:
	$(MAKE) -C driver/uio sync-hw-header KERNEL=$(KERNEL)

# ---------- EDF / Linux (Docker) ----------

edf-docker:
	$(MAKE) -C petalinux docker-build

edf-shell:
	$(MAKE) -C petalinux docker-shell

edf-setup:
	$(MAKE) -C petalinux edf-setup XSA=$(XSA)

edf-build:
	$(MAKE) -C petalinux edf-build MACHINE=$(MACHINE)

# ---------- MATLAB (HDL Coder) ----------

.PHONY: matlab-sim matlab-hdl matlab-cosim matlab-bitstream matlab-bench

matlab-sim:
	cd matlab && matlab -batch "run_matlab_tests"

matlab-hdl:
	cd matlab && matlab -batch "addpath('src','model','fpga'); export_repo_ip"

matlab-cosim:
	cd matlab && matlab -batch "addpath('src','test'); cd cosim; cosim_tb"

matlab-bitstream: matlab-hdl
	vivado -mode batch -source "fpga/tcl/build_vivado.tcl" -tclargs matlab "fpga/build"

matlab-bench:
	cd matlab && matlab -batch "addpath('src','test','bench'); benchmark_vi"

# ---------- Clean ----------

clean-edf:
	$(MAKE) -C petalinux clean

clean-fpga:
	$(MAKE) -C fpga clean

clean:
	$(MAKE) -C driver/uio clean
	$(MAKE) -C host clean
