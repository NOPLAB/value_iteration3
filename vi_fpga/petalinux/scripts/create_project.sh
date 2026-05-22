#!/bin/bash
# Create PetaLinux project for Ultra96-V2 and configure it.
# Run inside the Docker container with the repo mounted at /work.
#
# Usage: create_project.sh <path-to-xsa> [--bsp <path-to-bsp>]
set -euo pipefail

XSA="${1:?Usage: create_project.sh <path-to-xsa> [--bsp <path-to-bsp>]}"
shift

BSP=""
while [ $# -gt 0 ]; do
    case "$1" in
        --bsp) BSP="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

PROJECT_DIR="/work/petalinux/project/vi_petalinux"
REPO_ROOT="/work"

if [ -d "${PROJECT_DIR}" ]; then
    echo "Project already exists at ${PROJECT_DIR}. Skipping create."
else
    if [ -n "${BSP}" ]; then
        echo "==> Creating PetaLinux project from BSP: ${BSP}"
        petalinux-create -t project -s "${BSP}" \
            -n vi_petalinux \
            -p "${PROJECT_DIR%/*}"
    else
        echo "==> Creating PetaLinux project (generic zynqMP template)..."
        echo "    TIP: Use --bsp <avnet-ultra96v2-*.bsp> for better board support"
        petalinux-create -t project -n vi_petalinux \
            --template zynqMP \
            -p "${PROJECT_DIR%/*}"
    fi
fi

echo "==> Importing hardware description: ${XSA}"
cd "${PROJECT_DIR}"
petalinux-config --get-hw-description="$(dirname "${XSA}")" --silentconfig

# --- Device tree: add vi_sweep overlay ---
DT_FILES="${PROJECT_DIR}/project-spec/meta-user/recipes-bsp/device-tree/files"
DT_BBAPPEND="${PROJECT_DIR}/project-spec/meta-user/recipes-bsp/device-tree/device-tree.bbappend"

mkdir -p "${DT_FILES}"
cp "${REPO_ROOT}/driver/dts/vi_sweep.dtsi" "${DT_FILES}/"

# Add include to system-user.dtsi if not already present
if ! grep -q 'vi_sweep.dtsi' "${DT_FILES}/system-user.dtsi" 2>/dev/null; then
    echo '/include/ "vi_sweep.dtsi"' >> "${DT_FILES}/system-user.dtsi"
    echo "  Added vi_sweep.dtsi include to system-user.dtsi"
fi

# Add SRC_URI to bbappend if not already present
if ! grep -q 'vi_sweep.dtsi' "${DT_BBAPPEND}" 2>/dev/null; then
    echo 'SRC_URI += "file://vi_sweep.dtsi"' >> "${DT_BBAPPEND}"
    echo "  Added vi_sweep.dtsi to device-tree bbappend"
fi

echo "==> Project created and configured at ${PROJECT_DIR}"
echo "    Next: make -C petalinux plnx-build"
