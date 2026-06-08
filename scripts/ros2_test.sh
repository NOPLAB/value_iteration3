#!/usr/bin/env bash
set -euo pipefail

# The ROS setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so relax nounset while sourcing them, then restore it.
set +u
. /opt/ros/humble/setup.sh
. /ros2_rust_ws/install/local_setup.sh
set -u

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# Bridge / solver_factory / sweep_thread unit tests (no ROS deps).
cd "$REPO_ROOT/vi_ros2/vi_node"
cargo test --lib --no-default-features
cargo test --lib

# Oracle equivalence in serial mode (only runs under --no-default-features).
cargo test --test oracle_equivalence --no-default-features

# Full colcon build, then colcon test.
cd "$REPO_ROOT"
bash scripts/ros2_build.sh
cd "$REPO_ROOT/vi_ros2_ws"
colcon test --packages-select vi_node
colcon test-result --verbose
