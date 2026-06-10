#!/usr/bin/env bash
# vi_reference の u64 高速ソルバ群ハーネス (vi_u64_bench) をビルドし、house.pgm 上で
# 各ソルバを走らせて value_<solver>.npy / policy_<solver>.npy / timing_<solver>.json を
# /results に出力する。ref と同じく ROS 非経由・単スレッド。
#
# vi_reference は std のみ依存。リポジトリ直下の .cargo/config.toml (ROS patch) を拾うと
# Cargo.lock が汚染されるため、cargo の設定探索が効かない /tmp から --manifest-path でビルド
# する。専用 target を gitignore 済みキャッシュに置く (2 回目以降インクリメンタル)。
#
# Expects mounts: /workspace (new repo), /src_value_iteration (本家 maps, ro), /results
# 環境変数 SOLVERS でソルバ集合を上書き可 (既定は実装済みソルバ)。
set -e
source /opt/ros/humble/setup.sh 2>/dev/null || true

export CARGO_TARGET_DIR=/workspace/vi_compare/.cache/u64_target
cd /tmp
cargo build --release --manifest-path /workspace/vi_rs/Cargo.toml -p vi_reference --bin vi_u64_bench
BIN=$CARGO_TARGET_DIR/release/vi_u64_bench

PARAMS="${PARAMS:-/workspace/vi_compare/params.yaml}"
SOLVERS="${SOLVERS:-reference frontier3d frontier2d frontier_stack block_refine pyramid_sweep prio_ls prio_lc}"
for s in $SOLVERS; do
  echo "== u64 solver: $s =="
  python3 /workspace/vi_compare/u64/u64_bench.py \
    "$s" \
    "$PARAMS" \
    /src_value_iteration/maps/house.pgm \
    /results \
    "$BIN"
done
