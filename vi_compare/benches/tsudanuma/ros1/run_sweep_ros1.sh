#!/usr/bin/env bash
# 本家 vi_node (Node B) を lite インスタンス上で thread_num=m 掃引し、論文 §4.4.1 基準
# (ΔV<0.1s/sweep) での収束 wall-clock を記録する。論文 Fig.21 (収束時間 vs m) の本家側データ。
# コンテナ内実行。mounts: /src_value_iteration(ro), /workspace, /results。
#
# 本家 feedback の _delta は生 fixed-point 値 (1 s = PROB_BASE = 262144) として観測される。
# よって ΔV<0.1s は thr = 0.1*262144 = 26214 に対応。
set -e
source /opt/ros/noetic/setup.bash
TS=/workspace/vi_compare/benches/tsudanuma
LITE="${MAP_YAML:-/workspace/vi_compare/results/tsudanuma/lite/map_tsudanuma_lite.yaml}"
OUTDIR="${OUTDIR:-/results/tsudanuma/lite}"
mkdir -p "$OUTDIR"
TAG="${TAG:-}"   # 出力名タグ (Node B="" / Node B-2="_b2")。sweep CSV は衝突回避のため親へ。
SWEEP_CSV="${SWEEP_CSV:-/results/tsudanuma/sweep_ros1${TAG}.csv}"

DELTA_THR="${DELTA_THR:-26214}"   # 0.1 s in raw fixed-point
MAX_SWEEPS=100000
TIMEOUT="${TIMEOUT:-900}"
GOAL_X="${GOAL_X:-57.375}"; GOAL_Y="${GOAL_Y:-66.075}"; GOAL_YAW="${GOAL_YAW:-0}"
MLIST="${MLIST:-1 2 4 6 8 10 12 16}"

echo "[ros1 sweep] catkin_make 本家"
mkdir -p /catkin_ws/src
ln -sfn /src_value_iteration /catkin_ws/src/value_iteration
cd /catkin_ws
catkin_make >/tmp/catkin.log 2>&1 || { echo FAIL; tail -30 /tmp/catkin.log; exit 1; }
source devel/setup.bash

# 1 つの roscore を全 m で共有 (roslaunch は既存 roscore を検出して使う)
roscore >/tmp/roscore.log 2>&1 &
RC=$!
sleep 4

echo "m,sweeps,elapsed_sec,converged,resid_s,thread_num" > "$SWEEP_CSV"
for m in $MLIST; do
  echo "[ros1 sweep] m=$m ..."
  roslaunch "$TS/ros1/bench_tsudanuma.launch" map_yaml:=$LITE thread_num:=$m online:=${ONLINE:-false} \
    >"$OUTDIR/node_m${m}.log" 2>&1 &
  LP=$!
  sleep 1
  python3 "$TS/ros1/bench_client_tsudanuma.py" \
    $GOAL_X $GOAL_Y $GOAL_YAW $DELTA_THR $MAX_SWEEPS $TIMEOUT $m \
    "$OUTDIR/ros1${TAG}_m${m}" || echo "  (client m=$m returned nonzero)"
  kill $LP 2>/dev/null || true
  wait $LP 2>/dev/null || true
  pkill -9 -f 'value_iteration/vi_node' 2>/dev/null || true
  pkill -9 -f 'bin/map_server' 2>/dev/null || true
  # aggregate from per-m json (elapsed/sweeps/converged)
  python3 - "$OUTDIR/ros1${TAG}_m${m}.json" "$m" >> "$SWEEP_CSV" <<'PY'
import json,sys
j=json.load(open(sys.argv[1])); m=sys.argv[2]
d=j.get('last_max_delta'); rs=(d/262144.0) if d else float('nan')
print(f"{m},{j['sweeps']},{j['elapsed_sec']:.3f},{'Y' if j['converged'] else 'N'},{rs:.3f},{j['thread_num']}")
PY
  echo "  m=$m done: $(tail -1 "$SWEEP_CSV")"
  sleep 4
done
kill $RC 2>/dev/null || true
echo "=== ros1 sweep done ==="
cat "$SWEEP_CSV"
