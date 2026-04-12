.PHONY: driver host test-host test-hw \
       csim hls vivado bitstream sync-hw-header \
       plnx-docker plnx-shell plnx-create plnx-build plnx-package \
       clean clean-fpga clean-plnx

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

csim:
	$(MAKE) -C fpga csim

hls:
	$(MAKE) -C fpga hls

vivado: hls
	$(MAKE) -C fpga vivado

bitstream: vivado

sync-hw-header: hls
	$(MAKE) -C driver/uio sync-hw-header

# ---------- PetaLinux (Docker) ----------

plnx-docker:
	$(MAKE) -C petalinux docker-build

plnx-shell:
	$(MAKE) -C petalinux docker-shell

plnx-create:
	$(MAKE) -C petalinux plnx-create XSA=$(XSA)

plnx-build:
	$(MAKE) -C petalinux plnx-build

plnx-package:
	$(MAKE) -C petalinux plnx-package BITSTREAM=$(BITSTREAM)

# ---------- Clean ----------

clean-plnx:
	$(MAKE) -C petalinux clean

clean-fpga:
	$(MAKE) -C fpga clean

clean:
	$(MAKE) -C driver/uio clean
	$(MAKE) -C host clean
