#!/usr/bin/env bash
set -euo pipefail
: "${VI_TARGET_HOST:?set VI_TARGET_HOST}"
: "${VI_BIG_MAP_YAML:?set VI_BIG_MAP_YAML to the 700m campus map}"
: "${VI_BIG_GOAL:?set VI_BIG_GOAL as 'GX,GY[,GT]'}"

BIG_DIR=$(dirname "$VI_BIG_MAP_YAML")
PGM_NAME=$(awk '/^image:/ {print $2}' "$VI_BIG_MAP_YAML")

scp host/vi_cli "$VI_BIG_MAP_YAML" "$BIG_DIR/$PGM_NAME" \
    "$VI_TARGET_HOST":/tmp/

TARGET_YAML=$(basename "$VI_BIG_MAP_YAML")

ssh "$VI_TARGET_HOST" "
    cd /tmp &&
    /usr/bin/time -v ./vi_cli --map $TARGET_YAML \
        --goal $VI_BIG_GOAL \
        --threshold 0 --max-sweeps 50 -v
" 2>&1 | tee /tmp/vi_big.log

elapsed=$(grep -oP 'elapsed=\K[0-9.]+' /tmp/vi_big.log | tail -1)
echo "elapsed seconds: $elapsed"

if awk "BEGIN {exit !($elapsed < 60.0)}"; then
    echo "=== HW big-map test PASSED (under 60 s) ==="
else
    echo "=== HW big-map test FAILED (>= 60 s) ==="
    exit 1
fi
