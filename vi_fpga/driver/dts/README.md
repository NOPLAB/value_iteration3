# vi_sweep Device Tree overlay

The dtsi lives in the EDF layer:
`vi_fpga/petalinux/meta-vi-sweep/recipes-bsp/device-tree/files/vi_sweep.dtsi`,
appended to `system-user.dtsi` by `recipes-bsp/device-tree/device-tree.bbappend`
during the EDF/Yocto build (`make edf-build`). The same layer builds ikwzm's
u-dma-buf module (`recipes-kernel/u-dma-buf/u-dma-buf_git.bb`), and
`petalinux/scripts/setup.sh` adds `KERNEL_MODULE_AUTOLOAD += "uio_pdrv_genirq"`
to `local.conf`.

Note: the kernel only binds `compatible = "generic-uio"` nodes to
`uio_pdrv_genirq` when booted with `uio_pdrv_genirq.of_id=generic-uio` on the
kernel command line — without it no `/dev/uio*` appears.

## SPI interrupt numbers

The `interrupts = <0 N IRQ_TYPE_LEVEL_HIGH>` entries in `vi_sweep.dtsi` are the
GIC SPI numbers for `pl_ps_irq0[0]`/`[1]` (placeholders 89/90). Verify them
against the Vivado block design (or the generated `.xsa` interrupt table) after
regenerating the bitstream.

## Verification after boot

```
ls -l /dev/uio* /dev/udmabuf_value /dev/udmabuf_pendata
cat /sys/class/u-dma-buf/udmabuf_value/phys_addr
dmesg | grep -iE 'uio|udma'
```

`/sys/class/uio/uioN/name` must read `vi_sweep_cu0` / `vi_sweep_cu1` —
`vi_device_linux.c` looks the nodes up by these names.
