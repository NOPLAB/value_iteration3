#!/usr/bin/env bash
# Build workspace, launch vi_node with dump params, run rclpy client, shutdown.
# Expects mounts: /workspace (new repo), /src_value_iteration (本家 maps), /results
set -e
source /opt/ros/humble/setup.sh
source /ros2_rust_ws/install/local_setup.sh
cd /workspace
# Build the workspace. The cargo --release profile for vi_node is baked into
# scripts/ros2_build.sh (fair speed comparison vs the ROS1 C++ -O2 baseline), so
# no extra build args are passed here — any args here would be forwarded to
# colcon's --cmake-args, not --cargo-args.
bash scripts/ros2_build.sh
source /workspace/vi_ros2_ws/install/local_setup.sh
ros2 run vi_node vi_node --ros-args \
  --params-file /workspace/vi_compare/benches/house/ros2/vi_node_params.yaml &
NODE=$!
trap 'kill $NODE 2>/dev/null || true' EXIT
sleep 3
python3 /workspace/vi_compare/benches/house/ros2/bench_client.py \
  /workspace/vi_compare/benches/house/params.yaml \
  /src_value_iteration/maps/house.pgm \
  /results
# give vi_node a moment to finish writing npy after returning result
sleep 2
