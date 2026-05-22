SUMMARY = "u-dma-buf: User-space DMA buffer allocation driver"
HOMEPAGE = "https://github.com/ikwzm/u-dma-buf"
LICENSE = "BSD-2-Clause"
LIC_FILES_CHKSUM = "file://LICENSE;md5=bebf0492502927bef0741aa04d1f35f5"

SRC_URI = "git://github.com/ikwzm/u-dma-buf.git;branch=master;protocol=https"
SRCREV = "${AUTOREV}"
PV = "4.5+git"

S = "${WORKDIR}/git"

inherit module

RPROVIDES:${PN} += "kernel-module-u-dma-buf"
KERNEL_MODULE_AUTOLOAD += "u-dma-buf"
