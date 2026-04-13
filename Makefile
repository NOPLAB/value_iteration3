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

.PHONY: matlab-sim matlab-hdl matlab-cosim matlab-bitstream

matlab-sim:
	cd matlab && matlab -batch "addpath('src','testbench'); tb_full_sweep"

matlab-hdl:
	cd matlab && matlab -batch "addpath('src','model'); cd model; create_model"

matlab-cosim:
	cd matlab && matlab -batch "addpath('src','testbench'); cd cosim; cosim_tb"

matlab-bitstream:
	cd matlab && matlab -batch "addpath('src','testbench','model'); cd soc; build_bitstream"

# ---------- Clean ----------

clean-edf:
	$(MAKE) -C petalinux clean

clean-fpga:
	$(MAKE) -C fpga clean

clean:
	$(MAKE) -C driver/uio clean
	$(MAKE) -C host clean
