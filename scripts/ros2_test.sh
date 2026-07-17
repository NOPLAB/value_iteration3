#!/usr/bin/env bash
set -euo pipefail

# The ROS setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so relax nounset while sourcing them, then restore it.
set +u
. /opt/ros/humble/setup.sh
. /ros2_rust_ws/install/local_setup.sh
set -u

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# Library unit tests (no ROS deps): bridge / solver_factory / sweep_thread plus
# the oracle-equivalence tests (now library-internal — see src/oracle.rs). They
# run under `--lib` so cargo does NOT build the rclrs `vi_node` binary, which
# links only via colcon (a plain `cargo test --test ...` would fail to find the
# vi_interfaces C typesupport libs). Run both feature flavors.
cd "$REPO_ROOT/vi_ros2/vi_node"
cargo test --lib --no-default-features
cargo test --lib

# vi_planner core (rclrs-free) unit tests — same --lib technique.
cd "$REPO_ROOT/vi_ros2/vi_planner"
cargo test --lib

# Full colcon build (this is what links the rclrs `vi_node` binary; plain cargo
# cannot link the vi_interfaces C typesupport libs outside colcon).
cd "$REPO_ROOT"
bash scripts/ros2_build.sh
