#!/bin/bash
# Set up AMD EDF workspace and generate machine config from XSA.
# Run inside the Docker container with the repo mounted at /work.
#
# Usage: setup.sh [--xsa <path-to-xsa>]
set -euo pipefail

XSA=""
while [ $# -gt 0 ]; do
    case "$1" in
        --xsa) XSA="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

EDF_DIR="/work/edf"
BUILD_DIR="${EDF_DIR}/build"

# --- Step 1: repo init + sync ---
if [ ! -f "${EDF_DIR}/.repo/manifests/default-edf.xml" ]; then
    echo "==> Initializing EDF workspace..."
    mkdir -p "${EDF_DIR}"
    cd "${EDF_DIR}"
    repo init -u https://github.com/Xilinx/yocto-manifests.git \
        -b rel-v2025.2 -m default-edf.xml
    echo "==> Syncing layers (this may take a while)..."
    repo sync
else
    echo "==> EDF workspace already initialized. Running repo sync..."
    cd "${EDF_DIR}"
    repo sync
fi

# --- Step 2: Initialize build environment ---
echo "==> Initializing build environment..."
set +u
source edf-init-build-env build
set -u

# --- Step 3: Configure local.conf ---
LOCAL_CONF="${BUILD_DIR}/conf/local.conf"

# Shared sstate-cache and downloads for faster rebuilds
if ! grep -q 'SSTATE_DIR' "${LOCAL_CONF}" 2>/dev/null; then
    cat >> "${LOCAL_CONF}" << 'CONFEOF'

# --- VI Sweep project settings ---
SSTATE_DIR = "/work/sstate-cache"
DL_DIR = "/work/downloads"

# Save disk space: remove work dirs after packaging
INHERIT += "rm_work"

# UIO support in kernel
KERNEL_MODULE_AUTOLOAD += "uio_pdrv_genirq"
CONFEOF
    echo "  Updated local.conf with project settings"
fi

# --- Step 4: Add meta-vi-sweep layer ---
# Cannot use bitbake-layers on Windows FS (Unix socket not supported),
# so edit bblayers.conf directly.
BBLAYERS_CONF="${BUILD_DIR}/conf/bblayers.conf"
LAYER_DIR="/work/petalinux/meta-vi-sweep"
if [ -d "${LAYER_DIR}" ] && ! grep -q 'meta-vi-sweep' "${BBLAYERS_CONF}" 2>/dev/null; then
    sed -i "\|BBLAYERS|a\\  ${LAYER_DIR} \\\\" "${BBLAYERS_CONF}"
    echo "  Added meta-vi-sweep layer to bblayers.conf"
fi

# --- Step 5: Generate machine config from XSA (if provided) ---
if [ -n "${XSA}" ]; then
    echo "==> Generating machine config from XSA: ${XSA}"
    gen-machine-conf --soc-family zynqmp \
        --hw-description "${XSA}" \
        --machine-name ultra96v2-vi
    echo "  Machine 'ultra96v2-vi' configured"
    echo "  Set MACHINE=ultra96v2-vi in local.conf or on the bitbake command line"
fi

echo ""
echo "==> EDF setup complete."
echo "    Build with: MACHINE=ultra96v2-vi bitbake edf-linux-disk-image"
