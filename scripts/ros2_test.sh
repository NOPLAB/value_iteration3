#!/usr/bin/env bash
set -euo pipefail

# The ROS setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so relax nounset while sourcing them, then restore it.
set +u
. /opt/ros/humble/setup.sh
. /ros2_rust_ws/install/local_setup.sh
set -u

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# vi_planner core (rclrs-free) unit tests. They run under `--lib` so cargo does
# NOT build the rclrs `vi_planner` binary, which links only via colcon (a plain
# `cargo test --test ...` would fail to find the nav2_msgs C typesupport libs).
cd "$REPO_ROOT/vi_rs/vi_planner"
cargo test --lib

# Full colcon build (this is what links the rclrs `vi_planner` binary; plain
# cargo cannot link the message C typesupport libs outside colcon).
cd "$REPO_ROOT"
bash scripts/ros2_build.sh
