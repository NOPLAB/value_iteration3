.PHONY: driver host test-host test-hw \
       csim hls vivado bitstream sync-hw-header \
       clean clean-fpga

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
	$(MAKE) -C fpga/scripts csim

hls:
	$(MAKE) -C fpga/scripts hls

vivado: hls
	$(MAKE) -C fpga/scripts vivado

bitstream: vivado

sync-hw-header: hls
	$(MAKE) -C driver/uio sync-hw-header

# ---------- Clean ----------

clean-fpga:
	$(MAKE) -C fpga/scripts clean

clean:
	$(MAKE) -C driver/uio clean
	$(MAKE) -C host clean
