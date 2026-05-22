# vi_sweep Device Tree overlay

Include `vi_sweep.dtsi` from your Petalinux project's
`project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi`:

```dts
/include/ "vi_sweep.dtsi"
```

## Kernel configuration

Enable the following in `petalinux-config -c kernel`:

- `CONFIG_UIO=y`
- `CONFIG_UIO_PDRV_GENIRQ=y`
- `CONFIG_CMA_SIZE_MBYTES=1536` (or higher; `no-map` reserved-memory requires
  CMA headroom even though the buffers are carved out of CMA)

Add to bootargs (`petalinux-config`, subsystem → boot args):

```
uio_pdrv_genirq.of_id=generic-uio
```

Without this bootarg, the kernel does not associate `compatible = "generic-uio"`
device tree nodes with `uio_pdrv_genirq`, so `/dev/uio*` entries will not appear.

## u-dma-buf module

Add ikwzm's `u-dma-buf` as an external module under
`meta-user/recipes-modules/u-dma-buf/` following the upstream recipe at
https://github.com/ikwzm/udmabuf. On first boot, `modprobe u-dma-buf` (or add
to `/etc/modules`) and confirm the device nodes exist:

```
ls -l /dev/udmabuf_value /dev/udmabuf_pendata
cat /sys/class/u-dma-buf/udmabuf_value/phys_addr
cat /sys/class/u-dma-buf/udmabuf_pendata/phys_addr
```

## SPI interrupt numbers

The `interrupts = <0 N IRQ_TYPE_LEVEL_HIGH>` entries in `vi_sweep.dtsi` reference
the Zynq UltraScale+ GIC SPI number for `pl_ps_irq0[0]` and `pl_ps_irq0[1]`.
The exact numbers depend on the Vivado block design. Verify them by opening
the Vivado project, selecting the `zynq_ps` IP, and inspecting the interrupt
report, or by reading the generated `<design>.xsa` file's interrupt table.

Placeholder values in `vi_sweep.dtsi` are 89/90; update them if your build
assigns different numbers. See `fpga/vivado/ultra96v2/irq_notes.txt` for a
place to record the confirmed values alongside the Vivado project.

## Verification after boot

```
ls -l /dev/uio*
dmesg | grep -i uio
dmesg | grep -i udma
```

`uio0` and `uio1` should appear. Their `/sys/class/uio/uioN/name` must be
`vi_sweep_cu0` and `vi_sweep_cu1` (this is what `vi_device_linux.c` greps for).
