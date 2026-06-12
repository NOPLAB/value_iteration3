#!/usr/bin/env bash
# 本家 value_iteration を津田沼 0.15m プールマップ上で thread_num 並列スイープ実行し、
# 収束 wall-clock を /results/tsudanuma/ros1_parallel.{json,csv} に記録する。
# (vi_rs 側の bench_map --solver frontier2d_par と対をなす本家側ベンチ。)
#
# コンテナ内で実行する想定。mounts:
#   /src_value_iteration : 本家 repo (ro)
#   /workspace           : 新 repo (この repo)
#   /results             : 出力
#   /catkin_ws           : 永続ビルドキャッシュ (vi_compare/.cache/catkin_ws)
set -e
export DEBIAN_FRONTEND=noninteractive
source /opt/ros/noetic/setup.bash

THREAD_NUM="${THREAD_NUM:-16}"
GOAL_X="${GOAL_X:-147.375}"
GOAL_Y="${GOAL_Y:-100.125}"
GOAL_YAW="${GOAL_YAW:-0}"
DELTA_THR="${DELTA_THR:-0}"
MAX_SWEEPS="${MAX_SWEEPS:-100000}"
TIMEOUT="${TIMEOUT:-7200}"
SCALE="${SCALE:-3}"

TS=/workspace/vi_compare/benches/tsudanuma
OUTDIR=/results/tsudanuma
mkdir -p "$OUTDIR"

echo "[run_bench] pooling tsudanuma x${SCALE} -> /tmp/maps_processed (origin 0)"
python3 "$TS/maps/pool_tsudanuma.py" /workspace/assets/map_tsudanuma.pgm "$SCALE" /tmp/maps_processed

echo "[run_bench] catkin_make 本家"
mkdir -p /catkin_ws/src
ln -sfn /src_value_iteration /catkin_ws/src/value_iteration
cd /catkin_ws
catkin_make >/tmp/catkin.log 2>&1 || { echo "catkin_make FAILED"; tail -40 /tmp/catkin.log; exit 1; }
source devel/setup.bash

echo "[run_bench] launch vi_node (thread_num=${THREAD_NUM}) + map_server"
roslaunch "$TS/ros1/bench_tsudanuma.launch" \
  map_yaml:=/tmp/maps_processed/map_tsudanuma_015.yaml \
  thread_num:=${THREAD_NUM} >"$OUTDIR/ros1_node.log" 2>&1 &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT

echo "[run_bench] client: goal=(${GOAL_X},${GOAL_Y},${GOAL_YAW}) thr=${DELTA_THR} timeout=${TIMEOUT}s"
python3 "$TS/ros1/bench_client_tsudanuma.py" \
  "$GOAL_X" "$GOAL_Y" "$GOAL_YAW" "$DELTA_THR" "$MAX_SWEEPS" "$TIMEOUT" "$THREAD_NUM" \
  "$OUTDIR/ros1_parallel"

echo "[run_bench] done. results:"
cat "$OUTDIR/ros1_parallel.json"
