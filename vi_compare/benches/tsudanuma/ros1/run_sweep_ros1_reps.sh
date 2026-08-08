#!/usr/bin/env bash
# run_sweep_ros1.sh の n 回反復版: 各 m を REPS 回計測し、rep 列付き CSV を出力する。
# 論文 §4.4.1 と同じく各構成 10 試行の平均を取るための生データ。コンテナ内実行。
# mounts: /src_value_iteration(ro), /workspace, /catkin_ws, /results。
set -e
source /opt/ros/noetic/setup.bash
TS=/workspace/vi_compare/benches/tsudanuma
MAP="${MAP_YAML:?set MAP_YAML}"
OUTDIR="${OUTDIR:?set OUTDIR}"
mkdir -p "$OUTDIR"
SWEEP_CSV="${SWEEP_CSV:?set SWEEP_CSV}"

DELTA_THR="${DELTA_THR:-26214}"   # 0.1 s in raw fixed-point
MAX_SWEEPS=100000
TIMEOUT="${TIMEOUT:-120}"
GOAL_X="${GOAL_X:?set GOAL_X}"; GOAL_Y="${GOAL_Y:?set GOAL_Y}"; GOAL_YAW="${GOAL_YAW:-0}"
MLIST="${MLIST:-1 2 4 6 8 10 12 16}"
REPS="${REPS:-10}"

echo "[ros1 reps] catkin_make 本家"
mkdir -p /catkin_ws/src
rm -rf /catkin_ws/src/value_iteration
ln -s /src_value_iteration /catkin_ws/src/value_iteration
cd /catkin_ws
catkin_make >/tmp/catkin.log 2>&1 || { echo FAIL; tail -30 /tmp/catkin.log; exit 1; }
source devel/setup.bash

roscore >/tmp/roscore.log 2>&1 &
RC=$!
sleep 4

echo "m,rep,sweeps,elapsed_sec,converged,resid_s,thread_num" > "$SWEEP_CSV"
for m in $MLIST; do
  for r in $(seq 1 "$REPS"); do
    echo "[ros1 reps] m=$m rep=$r ..."
    roslaunch "$TS/ros1/bench_tsudanuma.launch" map_yaml:=$MAP thread_num:=$m online:=false \
      >"$OUTDIR/node_m${m}_r${r}.log" 2>&1 &
    LP=$!
    sleep 1
    python3 "$TS/ros1/bench_client_tsudanuma.py" \
      $GOAL_X $GOAL_Y $GOAL_YAW $DELTA_THR $MAX_SWEEPS $TIMEOUT $m \
      "$OUTDIR/ros1_m${m}_r${r}" || echo "  (client m=$m r=$r returned nonzero)"
    kill $LP 2>/dev/null || true
    wait $LP 2>/dev/null || true
    pkill -9 -f 'value_iteration/vi_node' 2>/dev/null || true
    pkill -9 -f 'bin/map_server' 2>/dev/null || true
    python3 - "$OUTDIR/ros1_m${m}_r${r}.json" "$m" "$r" >> "$SWEEP_CSV" <<'PY'
import json,sys
j=json.load(open(sys.argv[1])); m=sys.argv[2]; r=sys.argv[3]
d=j.get('last_max_delta'); rs=(d/262144.0) if d else float('nan')
print(f"{m},{r},{j['sweeps']},{j['elapsed_sec']:.3f},{'Y' if j['converged'] else 'N'},{rs:.3f},{j['thread_num']}")
PY
    echo "  m=$m rep=$r done: $(tail -1 "$SWEEP_CSV")"
    sleep 2
  done
done
kill $RC 2>/dev/null || true
echo "=== ros1 reps sweep done ==="
cat "$SWEEP_CSV"
