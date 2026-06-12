#!/usr/bin/env bash
# vi_rs frontier2d_par を lite インスタンス上で VI_THREADS=m 掃引し、収束 wall-clock を記録。
# 論文 Fig.21 (収束時間 vs m) / Fig.22 (speed rate vs m) の vi_rs 側データ。
set -e
cd /home/nop/dev/mywork/value_iteration_new
BM=vi_rs/target/release/bench_map
MAP="${MAP:-vi_compare/results/tsudanuma/lite/map_tsudanuma_lite.yaml}"
OUT="${OUT:-vi_compare/results/tsudanuma/sweep_vi_rs.csv}"
SCALE="${SCALE:-1}"
GOAL_X="${GOAL_X:-57.375}"; GOAL_Y="${GOAL_Y:-66.075}"; GOAL_THETA="${GOAL_THETA:-0}"
MLIST="${MLIST:-1 2 4 6 8 10 12 16}"
SOLVER="${SOLVER:-frontier2d_par}"
# 注: 負の座標を clap が flag と誤認しないよう --opt=value 形式。
COMMON="--map $MAP --scale $SCALE --solver $SOLVER --goal-x=$GOAL_X --goal-y=$GOAL_Y \
  --goal-theta-deg=$GOAL_THETA --goal-radius-m 0.30 --goal-margin-theta-deg 15 \
  --safety-radius-m 0.20 --safety-penalty 100000 --unknown obstacle --max-iters 2000000"

echo "m,iters,updates,total_ms,total_s,converged" > "$OUT"
for m in $MLIST; do
  echo "[vi_rs sweep] m=$m ..."
  line=$(VI_THREADS=$m $BM $COMMON 2>/dev/null | grep "$SOLVER |")
  # markdown row: | frontier2d_par | iters | updates | total_ms | total_s | conv |
  iters=$(echo "$line" | awk -F'|' '{gsub(/ /,"",$3);print $3}')
  upd=$(echo  "$line" | awk -F'|' '{gsub(/ /,"",$4);print $4}')
  ms=$(echo   "$line" | awk -F'|' '{gsub(/ /,"",$5);print $5}')
  s=$(echo    "$line" | awk -F'|' '{gsub(/ /,"",$6);print $6}')
  conv=$(echo "$line" | awk -F'|' '{gsub(/ /,"",$7);print $7}')
  echo "$m,$iters,$upd,$ms,$s,$conv" >> "$OUT"
  echo "  m=$m -> ${s}s iters=$iters converged=$conv"
done
echo "=== vi_rs sweep done ==="
cat "$OUT"
