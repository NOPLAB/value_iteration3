#!/usr/bin/env bash
# Build value_iteration, launch headless, run bench client, shutdown.
# Expects mounts: /src_value_iteration (本家, ro), /workspace (new repo), /results
set -e
source /opt/ros/noetic/setup.bash
mkdir -p /catkin_ws/src
ln -sfn /src_value_iteration /catkin_ws/src/value_iteration
cd /catkin_ws
# /catkin_ws はホスト側 vi_compare/.cache/catkin_ws にマウントして永続化する。
# catkin_make は増分ビルド: 初回はフルビルド、以降は本家 C++ が未変更ならほぼ no-op
# (devel/build がキャッシュされるため毎回の再コンパイルを避けられる)。
catkin_make
source devel/setup.bash
roslaunch /workspace/vi_compare/benches/house/ros1/bench.launch \
  map_yaml:=/src_value_iteration/maps/house.yaml &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT
# 第1引数で params を差し替え可能 (strict 比較は delta_threshold=-1, max_sweeps=F の params)。
PARAMS="${1:-/workspace/vi_compare/benches/house/params.yaml}"
python3 /workspace/vi_compare/benches/house/ros1/bench_client.py \
  "$PARAMS" /results
