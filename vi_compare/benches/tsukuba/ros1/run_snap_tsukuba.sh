#!/usr/bin/env bash
# 本家 value_iteration (snapshot パッチ版) を tsukuba 0.15m プールマップ
# (4417x2367x60 = 627M states) 上で thread_num 並列スイープし、収束しないまま
# TIMEOUT まで回しつつ snapshotWorker で min-θ 値場を周期ダンプする。
# vi_rs frontier2d_sparse の VI_SNAP_DIR ランと対をなす本家側の動画素材生成。
#
# コンテナ内で実行する想定。mounts:
#   /src_value_iteration : 本家 value_iteration (無改変, ro)。snapshotWorker は
#                          video/snapshot.patch をコピーに適用して足す。
#   /workspace           : この repo (results もこの下)
#   /catkin_ws           : 永続ビルドキャッシュ (vi_compare/.cache/catkin_ws)
# env:
#   VI_SNAP_DIR : snap_*.bin + times.csv の出力先 (roslaunch 前に export 必須)
#   VI_SNAP_MS  : スナップショット周期 [ms]
set -e
export DEBIAN_FRONTEND=noninteractive
source /opt/ros/noetic/setup.bash

THREAD_NUM="${THREAD_NUM:-16}"
# goal: vi_rs 側 bench_map と同一 (world 座標, real origin -553.84/-60.609)
GOAL_X="${GOAL_X:-20.5}"
GOAL_Y="${GOAL_Y:--1.0}"
GOAL_YAW="${GOAL_YAW:-0}"
DELTA_THR="${DELTA_THR:-0}"      # 0 = 収束しきい raw fixed-point (627M では TIMEOUT まで未収束)
MAX_SWEEPS="${MAX_SWEEPS:-100000}"
TIMEOUT="${TIMEOUT:-600}"
GOAL_MARGIN_RADIUS="${GOAL_MARGIN_RADIUS:-0.5}"
GOAL_MARGIN_THETA="${GOAL_MARGIN_THETA:-180}"
export GOAL_MARGIN_RADIUS GOAL_MARGIN_THETA

TS=/workspace/vi_compare/benches/tsukuba
OUTDIR=/workspace/vi_compare/results/tsukuba
MAP_YAML="${MAP_YAML:-$OUTDIR/map_tsukuba_pooled.yaml}"
export MAP_YAML
OUT_PREFIX="${OUT_PREFIX:-ros1_m16}"
mkdir -p "$OUTDIR/frames_ros1" "$OUTDIR/snap_run"

export VI_SNAP_DIR="${VI_SNAP_DIR:-$OUTDIR/frames_ros1}"
export VI_SNAP_MS="${VI_SNAP_MS:-2000}"
echo "[run_snap] VI_SNAP_DIR=$VI_SNAP_DIR VI_SNAP_MS=$VI_SNAP_MS"

echo "[run_snap] catkin_make 本家 (snapshot パッチ適用)"
mkdir -p /catkin_ws/src
# :ro マウントへはパッチを当てられないのでコピーしてから snapshot.patch を適用する。
rm -rf /catkin_ws/src/value_iteration
cp -r /src_value_iteration /catkin_ws/src/value_iteration
patch -p1 -d /catkin_ws/src/value_iteration < /workspace/vi_compare/video/snapshot.patch
cd /catkin_ws
catkin_make >/tmp/catkin.log 2>&1 || { echo "catkin_make FAILED"; tail -40 /tmp/catkin.log; exit 1; }
source devel/setup.bash

echo "[run_snap] launch vi_node (thread_num=${THREAD_NUM}) + map_server"
roslaunch "$TS/ros1/bench_tsukuba.launch" \
  map_yaml:="$MAP_YAML" \
  goal_margin_radius:=${GOAL_MARGIN_RADIUS} \
  goal_margin_theta:=${GOAL_MARGIN_THETA} \
  thread_num:=${THREAD_NUM} >"$OUTDIR/snap_run/${OUT_PREFIX}_node.log" 2>&1 &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT

echo "[run_snap] client: goal=(${GOAL_X},${GOAL_Y},${GOAL_YAW}) margin=(${GOAL_MARGIN_RADIUS}m,${GOAL_MARGIN_THETA}deg) thr=${DELTA_THR} timeout=${TIMEOUT}s"
python3 "$TS/ros1/bench_client_tsukuba.py" \
  "$GOAL_X" "$GOAL_Y" "$GOAL_YAW" "$DELTA_THR" "$MAX_SWEEPS" "$TIMEOUT" "$THREAD_NUM" \
  "$OUTDIR/snap_run/${OUT_PREFIX}"

echo "[run_snap] done. results:"
cat "$OUTDIR/snap_run/${OUT_PREFIX}.json"
echo
echo "[run_snap] frames: $(ls "$VI_SNAP_DIR"/snap_*.bin 2>/dev/null | wc -l)"
