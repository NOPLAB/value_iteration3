#!/usr/bin/env bash
# Build value_iteration, launch headless, run bench client, shutdown.
# Expects mounts: /src_value_iteration (本家, ro), /workspace (new repo), /results
set -e
source /opt/ros/noetic/setup.bash
mkdir -p /catkin_ws/src
ln -sfn /src_value_iteration /catkin_ws/src/value_iteration
cd /catkin_ws
if [ ! -f devel/setup.bash ]; then
  catkin_make
fi
source devel/setup.bash
roslaunch /workspace/vi_compare/ros1/bench.launch \
  map_yaml:=/src_value_iteration/maps/house.yaml &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT
python3 /workspace/vi_compare/ros1/bench_client.py \
  /workspace/vi_compare/params.yaml /results
