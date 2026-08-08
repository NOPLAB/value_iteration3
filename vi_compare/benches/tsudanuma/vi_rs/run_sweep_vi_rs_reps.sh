#!/usr/bin/env bash
# bench_map の m 掃引を各 m につき REPS 回計測し、rep 列付き CSV を出力する。
# 統計 (mean±std) は後段 (aggregate_stats.py) で集計する。VI_THREADS=m で bench_map を回す。
set -e
cd "${REPO_ROOT:-$(cd "$(dirname "$0")/../../../.." && pwd)}"
BM=vi_rs/target/release/bench_map
MAP="${MAP:?set MAP}"
OUT="${OUT:?set OUT}"
SCALE="${SCALE:-1}"
GOAL_X="${GOAL_X:?set GOAL_X}"; GOAL_Y="${GOAL_Y:?set GOAL_Y}"; GOAL_THETA="${GOAL_THETA:-0}"
MLIST="${MLIST:-1 2 4 6 8 10 12 16}"
REPS="${REPS:-10}"
SOLVER="${SOLVER:-frontier2d_sparse}"
EXTRA="${EXTRA:-}"
COMMON="--map $MAP --scale $SCALE --solver $SOLVER --goal-x=$GOAL_X --goal-y=$GOAL_Y \
  --goal-theta-deg=$GOAL_THETA --goal-radius-m 0.30 --goal-margin-theta-deg 15 \
  --safety-radius-m 0.20 --safety-penalty 100000 --unknown obstacle --max-iters 2000000 $EXTRA"

echo "m,rep,iters,updates,total_ms,total_s,converged" > "$OUT"
for m in $MLIST; do
  for r in $(seq 1 "$REPS"); do
    line=$(VI_THREADS=$m $BM $COMMON 2>/dev/null | grep "$SOLVER |")
    iters=$(echo "$line" | awk -F'|' '{gsub(/ /,"",$3);print $3}')
    upd=$(echo  "$line" | awk -F'|' '{gsub(/ /,"",$4);print $4}')
    ms=$(echo   "$line" | awk -F'|' '{gsub(/ /,"",$5);print $5}')
    s=$(echo    "$line" | awk -F'|' '{gsub(/ /,"",$6);print $6}')
    conv=$(echo "$line" | awk -F'|' '{gsub(/ /,"",$7);print $7}')
    echo "$m,$r,$iters,$upd,$ms,$s,$conv" >> "$OUT"
    echo "  m=$m rep=$r -> ${s}s converged=$conv"
  done
done
echo "=== vi_rs reps sweep done ==="
