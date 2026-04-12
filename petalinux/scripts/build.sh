#!/bin/bash
# Build AMD EDF Linux image for Ultra96-V2.
# Run inside the Docker container with the repo mounted at /work.
#
# Usage: build.sh [--machine <name>] [--image <recipe>]
set -euo pipefail

MACHINE="${MACHINE:-ultra96v2-vi}"
IMAGE="edf-linux-disk-image"

while [ $# -gt 0 ]; do
    case "$1" in
        --machine) MACHINE="$2"; shift 2 ;;
        --image)   IMAGE="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

EDF_DIR="/work/edf"
OUTPUT_DIR="/work/petalinux/output"

if [ ! -f "${EDF_DIR}/build/conf/local.conf" ]; then
    echo "ERROR: EDF not set up. Run setup.sh first."
    exit 1
fi

cd "${EDF_DIR}"
set +u
source edf-init-build-env build
set -u

echo "==> Building: MACHINE=${MACHINE} bitbake ${IMAGE}"
MACHINE="${MACHINE}" bitbake "${IMAGE}"

# Copy key artifacts
echo "==> Copying artifacts to ${OUTPUT_DIR}..."
mkdir -p "${OUTPUT_DIR}"
DEPLOY="${EDF_DIR}/build/tmp/deploy/images/${MACHINE}"

for f in "${DEPLOY}"/*.wic.gz "${DEPLOY}"/BOOT.BIN "${DEPLOY}"/image.ub \
         "${DEPLOY}"/boot.scr "${DEPLOY}"/*.dtb; do
    [ -f "$f" ] && cp "$f" "${OUTPUT_DIR}/" && echo "  $(basename "$f")"
done

echo "==> Done. Artifacts in ${OUTPUT_DIR}/"
ls -lh "${OUTPUT_DIR}/"
