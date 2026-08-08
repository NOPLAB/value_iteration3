#!/usr/bin/env bash
# Sequential ROS1 -> ref(vi_reference) -> compare. Run from repo root.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ORIG="${VI_ORIG:-$(cd "$REPO_ROOT/.." && pwd)/value_iteration}"
RESULTS="$REPO_ROOT/vi_compare/results/house_oracle"
# 本家 catkin ビルドの永続キャッシュ (--rm コンテナ間で /catkin_ws を保持し再コンパイルを回避)。
# .cache 配下は Docker(root) が作成するので host では触らない。
CATKIN_CACHE="$REPO_ROOT/vi_compare/.cache/catkin_ws"
mkdir -p "$RESULTS"

echo "== [1/3] ROS1 (本家) =="
docker run --rm \
  -v "$ORIG":/src_value_iteration:ro \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  -v "$CATKIN_CACHE":/catkin_ws \
  vi_compare_ros1:noetic \
  bash /workspace/vi_compare/benches/house/ros1/run_ros1_bench.sh

echo "== [2/3] ref (vi_reference u64 忠実移植) =="
docker run --rm \
  -v "$ORIG":/src_value_iteration:ro \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_ros2_dev:humble \
  bash /workspace/vi_compare/benches/house/vi_rs/run_ref_bench.sh

echo "== [3/3] compare (ref を本家と比較) =="
docker run --rm \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_compare_ros1:noetic \
  bash -lc "cd /workspace/vi_compare/benches/house/compare && python3 compare.py /results ref"

echo "reports: $RESULTS/report_ref.md (vs vi_reference u64)"
