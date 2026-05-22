#!/usr/bin/env bash
set -euo pipefail
: "${VI_TARGET_HOST:?set VI_TARGET_HOST to the Ultra96 hostname}"

TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

python3 host/test/hw/make_tiny_map.py "$TMPDIR"

# Copy CLI + map to target
scp host/vi_cli "$TMPDIR/smoke.pgm" "$TMPDIR/smoke.yaml" \
    "$VI_TARGET_HOST":/tmp/

ssh "$VI_TARGET_HOST" '
    cd /tmp &&
    ./vi_cli --map smoke.yaml --goal 35,20 --verify \
             --threshold 0 --max-sweeps 100
' | tee "$TMPDIR/smoke.log"

if grep -q "verify: PASS" "$TMPDIR/smoke.log"; then
    echo "=== HW smoke test PASSED ==="
else
    echo "=== HW smoke test FAILED ==="
    exit 1
fi
