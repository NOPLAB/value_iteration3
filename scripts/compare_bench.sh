#!/usr/bin/env bash
# Sequential ROS1 -> ROS2 -> compare. Run from repo root.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ORIG="${VI_ORIG:-$(cd "$REPO_ROOT/.." && pwd)/value_iteration}"
RESULTS="$REPO_ROOT/vi_compare/results"
mkdir -p "$RESULTS"

echo "== [1/3] ROS1 (本家) =="
docker run --rm \
  -v "$ORIG":/src_value_iteration:ro \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_compare_ros1:noetic \
  bash /workspace/vi_compare/ros1/run_ros1_bench.sh

echo "== [2/3] ROS2 (vi_node) =="
docker run --rm \
  -v "$ORIG":/src_value_iteration:ro \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_ros2_dev:humble \
  bash /workspace/vi_compare/ros2/run_ros2_bench.sh

echo "== [3/3] compare =="
docker run --rm \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_compare_ros1:noetic \
  bash -lc "cd /workspace/vi_compare/compare && python3 compare.py /results"

echo "report: $RESULTS/report.md"
