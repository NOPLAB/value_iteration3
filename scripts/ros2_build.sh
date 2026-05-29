#!/usr/bin/env bash
set -euo pipefail

. /opt/ros/humble/setup.sh
. /ros2_rust_ws/install/local_setup.sh

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO_ROOT/vi_ros2_ws"
mkdir -p "$WS/src"
ln -sfn "$REPO_ROOT/vi_ros2/vi_interfaces" "$WS/src/vi_interfaces"
ln -sfn "$REPO_ROOT/vi_ros2/vi_node"       "$WS/src/vi_node"

cd "$WS"
colcon build --packages-select vi_interfaces vi_node \
       --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
