#!/usr/bin/env bash
# 本家(ROS1) vs ref(vi_reference) を「真の固定点」で bit 比較する strict ベンチ。
#
# 本家の通常収束 (delta>>18==0) は確率的アクションのサブステップ精細化を残し、停止スイープ数に
# 依存して値が僅かに変わる (RMSE~0.5)。そこで:
#   1) ref を strict モード (delta_threshold<0) で到達可能セルが変化しなくなる固定点 F まで回す。
#   2) 本家を delta_threshold=-1 (soft 停止させず) ・max_sweeps=F で F スイープ回し固定点へ。
#   3) compare.py で比較 → 忠実なら bit 一致するはず。
# Run from repo root.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ORIG="${VI_ORIG:-$(cd "$REPO_ROOT/.." && pwd)/value_iteration}"
RESULTS="$REPO_ROOT/vi_compare/results/house_oracle"
CATKIN_CACHE="$REPO_ROOT/vi_compare/.cache/catkin_ws"
mkdir -p "$RESULTS"   # .cache 配下は Docker(root) が作成するので host では触らない
# strict params はホストが書くため /tmp (host 書込可) に置きコンテナへマウントする。
STRICT_REF=/tmp/vi_params_strict_ref.yaml
STRICT_ROS1=/tmp/vi_params_strict_ros1.yaml

# strict ref params: delta_threshold=-1 (ハーネス strict), max_sweeps=2000 (上限)
sed -e 's/^\( *delta_threshold:\).*/\1 -1/' -e 's/^\( *max_sweeps:\).*/\1 2000/' \
    "$REPO_ROOT/vi_compare/benches/house/params.yaml" > "$STRICT_REF"

echo "== [1/3] ref strict (到達可能セルの真の固定点まで) =="
docker run --rm \
  -v "$ORIG":/src_value_iteration:ro \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  -v "$STRICT_REF":/params_strict.yaml:ro \
  vi_ros2_dev:humble \
  bash /workspace/vi_compare/benches/house/vi_rs/run_ref_bench.sh /params_strict.yaml

F=$(python3 -c "import json;print(int(json.load(open('$RESULTS/timing_ref.json'))['sweeps']))")
echo "ref reached reachable fixed point at F=$F sweeps"

# strict ros1 params: delta_threshold=-1 (soft 停止せず), max_sweeps=F → 本家を F スイープ
sed -e 's/^\( *delta_threshold:\).*/\1 -1/' -e "s/^\( *max_sweeps:\).*/\1 $F/" \
    "$REPO_ROOT/vi_compare/benches/house/params.yaml" > "$STRICT_ROS1"

echo "== [2/3] ros1 strict (本家を F=$F スイープ = 固定点) =="
docker run --rm \
  -v "$ORIG":/src_value_iteration:ro \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  -v "$CATKIN_CACHE":/catkin_ws \
  -v "$STRICT_ROS1":/params_strict.yaml:ro \
  vi_compare_ros1:noetic \
  bash /workspace/vi_compare/benches/house/ros1/run_ros1_bench.sh /params_strict.yaml

echo "== [3/3] compare (本家固定点 vs ref固定点) =="
docker run --rm \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_compare_ros1:noetic \
  bash -lc "cd /workspace/vi_compare/benches/house/compare && python3 compare.py /results ref"

echo "report: $RESULTS/report_ref.md (strict / 固定点 bit 比較)"
