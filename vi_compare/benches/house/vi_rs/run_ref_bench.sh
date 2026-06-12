#!/usr/bin/env bash
# vi_reference (本家 u64 忠実移植) ハーネスをビルドして house.pgm 上で走らせ、
# value_ref.npy / policy_ref.npy / timing_ref.json を /results に出力する。
# Expects mounts: /workspace (new repo), /src_value_iteration (本家 maps, ro), /results
set -e
source /opt/ros/humble/setup.sh 2>/dev/null || true

# vi_reference は std のみ依存。リポジトリ直下の .cargo/config.toml (ROS patch) を
# 拾うと Cargo.lock に [[patch.unused]] が書き込まれ汚染されるため、cargo の設定探索が
# 効かない /tmp から --manifest-path でビルドする (patch 不適用・警告なし・Cargo.lock 不変)。
# ホストの vi_rs/target と混ざらないよう専用 target を gitignore 済みキャッシュに置く
# (2 回目以降はインクリメンタルにキャッシュされる)。
export CARGO_TARGET_DIR=/workspace/vi_compare/.cache/ref_target
cd /tmp
cargo build --release --manifest-path /workspace/vi_rs/Cargo.toml -p vi_reference --bin vi_ref_bench
BIN=$CARGO_TARGET_DIR/release/vi_ref_bench

# 第1引数で params を差し替え可能 (strict 比較は delta_threshold<0 の params を渡す)。
PARAMS="${1:-/workspace/vi_compare/benches/house/params.yaml}"
python3 /workspace/vi_compare/benches/house/vi_rs/ref_bench.py \
  "$PARAMS" \
  /src_value_iteration/maps/house.pgm \
  /results \
  "$BIN"
