#!/usr/bin/env bash
# vi_rs Frontier3D 直接ハーネス (vi_f3d_bench) をビルドして house.pgm 上で走らせ、
# value_f3d.npy / policy_f3d.npy / timing_f3d.json を /results に出力する。
#
# ref (vi_reference) と対をなす ROS 非経由・単スレッドのハーネス。vi_node の bridge を
# 再利用して VIContext を構築するため、ビルドには colcon が生成する .cargo/config.toml
# (rclrs 等の [patch.crates-io] リダイレクト) が必要。よって まず scripts/ros2_build.sh で
# config を用意し依存をビルドしてから、vi_f3d_bench を明示ビルドする。
#
# Expects mounts: /workspace (new repo), /src_value_iteration (本家 maps, ro), /results
set -euo pipefail
set +u
. /opt/ros/humble/setup.sh
. /ros2_rust_ws/install/local_setup.sh
set -u

REPO_ROOT=/workspace
WS="$REPO_ROOT/vi_ros2_ws"

# colcon で vi_interfaces/vi_node をビルド (=.cargo/config.toml 生成 + 依存クレートをコンパイル)。
bash "$REPO_ROOT/scripts/ros2_build.sh"

# vi_f3d_bench を明示ビルド (colcon の target-dir を再利用 → 依存は再コンパイルしない)。
# cwd を vi_node ソース直下にすると、cargo が上方探索で $REPO_ROOT/.cargo/config.toml を拾う。
cd "$REPO_ROOT/vi_ros2/vi_node"
cargo build --release --bin vi_f3d_bench --target-dir "$WS/build/vi_node"
BIN="$WS/build/vi_node/release/vi_f3d_bench"
test -x "$BIN" || { echo "ERROR: vi_f3d_bench not built at $BIN" >&2; exit 1; }

# 第1引数で params を差し替え可能 (strict 比較等)。
PARAMS="${1:-/workspace/vi_compare/params.yaml}"
python3 /workspace/vi_compare/f3d/f3d_bench.py \
  "$PARAMS" \
  /src_value_iteration/maps/house.pgm \
  /results \
  "$BIN"
