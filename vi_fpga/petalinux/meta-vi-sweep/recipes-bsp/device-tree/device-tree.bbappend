# Add vi_sweep UIO + u-dma-buf device tree overlay
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI += "file://vi_sweep.dtsi"

do_configure:append() {
    cat ${UNPACKDIR}/vi_sweep.dtsi >> ${B}/system-user.dtsi
}
