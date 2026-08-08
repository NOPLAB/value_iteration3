#!/usr/bin/env bash
# =============================================================================
# 統一条件ベンチマーク一括実行スクリプト（report_paper_style.md の採用データを再現）
#
#   usage: bash vi_compare/benches/run_all_benchmarks.sh [target...]
#     target: house | lite | fullext | tsukuba | all（既定 all）
#             fullassets（参考行用・all には含まれない）
#   env:
#     REPS=10        vi_rs 側の反復回数（house は ROS1 側も REPS 反復）
#     VIRS_ONLY=1    vi_rs（提案手法）側のみ実行
#     ROS1_ONLY=1    ROS1（本家）側のみ実行
#     FORCE=1        メモリ残量チェックを無視（筑波）
#
# 全マップ完全同一条件（論文 Ueda 2023 Table 1/2 準拠）:
#   goal 半径 0.3 m / ±15°、安全半径 0.2 m / penalty 10^5 s (=R_max)、θ=60 (6°)、
#   行動 6 種、Δt=1 s、18bit 固定小数 u64、unknown セルは obstacle 扱い。
#   収束判定: 本家 = ΔV<0.1 s/sweep（raw delta_threshold=26214）、
#             vi_rs = フロンティア消滅（厳密固定点 ΔV=0、より強い条件）。
#   ROS1 の計時はクライアントの goal 送信から（states 構築を含む — 全マップ同一慣例）。
#
# マップと goal（採用値）:
#   house     384x384x60 (0.05 m, 自由 2.34M)                goal ( 6.0,   -2.0,  90°)
#   lite      540x540x60 (0.15 m, 自由 9.73M = 論文の 98.2%) goal (57.375, 66.075, 0°)
#   fullext   1963x1334x60 (0.15 m, 総 157M/自由 9.73M      goal (164.175,125.625, 0°)
#             = 論文の探索条件一致: 全体地図確保・自由のみ更新)
#   tsukuba   4417x2367x60 (0.15 m, 総 627M/自由 71.3M)      goal (20.5,   -1.0,   0°)
#   fullassets 1963x1334x60 (0.15 m, 自由 44M = 黒線なし・参考) goal (179.9,  43.7,   0°)
#
# ソルバ: house/lite/fullext/fullassets = frontier2d_sparse
#         tsukuba = frontier2d_sparse_compact（plain sparse は常駐 ~59 GB で
#         WSL 54 GiB に載らないため。収束値は bit-exact、band=auto, RAM sink）
#
# 実行は全ステップ直列（計測汚染防止）。所要目安（Ryzen 9700X/64GB）:
#   house ~50 min / lite ~2.5 h / fullext ~1.8 h / tsukuba ~1.7 h → all ~6.5 h
#
# 出力（既存ファイルは .bak.<timestamp> に退避してから上書き）:
#   results/house/sweep_vi_rs_sparse_house_x<REPS>.csv, sweep_ros1_house_x<REPS>.csv（ファイル名は REPS に追従）
#   results/tsudanuma/sweep_vi_rs_sparse_lite_x${REPS}.csv, sweep_ros1.csv
#   results/tsudanuma/sweep_vi_rs_sparse_fullext_x${REPS}.csv, sweep_ros1_fullext.csv
#   results/tsukuba/sweep_vi_rs_compact_015_x${REPS}.csv, snap_run/ros1_015_g15_m16_t900.*
# =============================================================================
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd "$SCRIPT_DIR/../.." && pwd)
cd "$ROOT"

REPS="${REPS:-10}"
DOCKER_IMG="${VI_COMPARE_DOCKER_IMG:-vi_compare_ros1:noetic}"
# 本家 value_iteration の checkout (無改変)。snapshotWorker が要るステップは
# コンテナ内で vi_compare/video/snapshot.patch をコピーへ適用する。
VI_ORIG="${VI_ORIG:-$ROOT/../value_iteration}"
REPS_VIRS="$SCRIPT_DIR/tsudanuma/vi_rs/run_sweep_vi_rs_reps.sh"
REPS_ROS1="$SCRIPT_DIR/tsudanuma/ros1/run_sweep_ros1_reps.sh"
SNAP_TSUKUBA="$SCRIPT_DIR/tsukuba/ros1/run_snap_tsukuba.sh"
AGG="$SCRIPT_DIR/aggregate_stats.py"
BM=vi_rs/target/release/bench_map
DELTA_THR=26214   # 0.1 s in 18bit raw

log() { echo "[bench $(date +%H:%M:%S)] $*"; }
bak() { if [ -f "$1" ]; then cp "$1" "$1.bak.$(date +%Y%m%d-%H%M%S)"; log "backup: $1"; fi; }

# CSV 先頭に条件コメントを後付けする（reps スクリプトはプレーン CSV を書くため）
prepend_hdr() { # prepend_hdr FILE <<EOF ... EOF
  local f="$1" tmp; tmp=$(mktemp)
  cat - "$f" > "$tmp" && mv "$tmp" "$f"
}

preflight_virs() {
  if [ ! -x "$BM" ]; then
    log "bench_map をビルド（repo 外から --manifest-path: root .cargo の ROS パッチ回避）"
    (cd /tmp && cargo build --release --manifest-path "$ROOT/vi_rs/Cargo.toml" -p vi_bench --bin bench_map)
  fi
}

preflight_ros1() {
  docker image inspect "$DOCKER_IMG" >/dev/null 2>&1 || {
    echo "ERROR: docker image $DOCKER_IMG がありません（vi_compare/docker 参照）" >&2; exit 1; }
}

preflight_fullext_map() {
  if [ ! -f vi_compare/results/tsudanuma/fullext/map_tsudanuma_fullext.yaml ]; then
    log "fullext マップを生成（embed_fullext.py）"
    python3 "$SCRIPT_DIR/tsudanuma/maps/embed_fullext.py" \
      vi_compare/results/tsudanuma/full/map_tsudanuma_015.pgm \
      vi_compare/results/tsudanuma/lite/map_tsudanuma_lite.pgm \
      vi_compare/results/tsudanuma/fullext
  fi
}

mem_guard() { # mem_guard MIN_AVAILABLE_MB LABEL
  local avail; avail=$(free -m | awk '/^Mem:/{print $7}')
  if [ "$avail" -lt "$1" ] && [ "${FORCE:-0}" != "1" ]; then
    echo "ERROR: available ${avail} MB < $1 MB（$2）。他プロセス終了後に再実行するか FORCE=1。" >&2
    exit 1
  fi
}

docker_ros1() { # docker_ros1 NAME SCRIPT_IN_CONTAINER LOG_HOST_PATH env...
  local name="$1" script="$2" hostlog="$3"; shift 3
  local envs=(); local e
  for e in "$@"; do envs+=(-e "$e"); done
  trap 'docker rm -f '"$name"' >/dev/null 2>&1 || true' INT TERM
  docker run --rm --name "$name" \
    -v "$VI_ORIG:/src_value_iteration:ro" \
    -v "$ROOT:/workspace" \
    -v "$ROOT/vi_compare/.cache/catkin_ws:/catkin_ws" \
    -v "$ROOT/vi_compare/results:/results" \
    "${envs[@]}" "$DOCKER_IMG" \
    bash -c "bash $script > $hostlog 2>&1"
  trap - INT TERM
}

# ---------------------------------------------------------------- bench -----
# 全ターゲット共通の実行器 (vi_rs 側 → ROS1 側の順、直列)。map/goal/solver/
# MLIST/TIMEOUT/出力先/キャプションは冒頭の case 表で選ぶ。ROS1 側は
# run_sweep_ros1_reps.sh に一本化 (単発は REPS=1; CSV に rep 列が付くが
# aggregate_stats.py は両対応)。tsukuba の ROS1 側のみ snap スクリプト。
bench() {
  local t="$1"
  local map gx gy gt solver virs_mlist virs_out virs_log virs_hdr
  local ros1_reps ros1_timeout ros1_mlist ros1_csv ros1_outdir ros1_runlog ros1_log ros1_hdr=""
  case "$t" in
    house)
      map=vi_compare/results/house/house.yaml
      gx=6.0 gy=-2.0 gt=90 solver=frontier2d_sparse virs_mlist=""
      virs_out=vi_compare/results/house/sweep_vi_rs_sparse_house_x${REPS}.csv
      virs_log="house vi_rs sparse n=$REPS (m=1..16)"
      virs_hdr="# house (384x384x60, 0.05m, 自由 2.34M) frontier2d_sparse n=$REPS 反復掃引
# goal (6.0,-2.0,90°), 0.3m/±15°, 安全0.2m/1e5, unknown=obstacle, 18bit u64。VI_THREADS=m"
      ros1_reps=$REPS ros1_timeout=120 ros1_mlist=""
      ros1_csv=vi_compare/results/house/sweep_ros1_house_x${REPS}.csv
      ros1_outdir=/results/house/x${REPS}_logs
      ros1_runlog=/workspace/vi_compare/results/house/run_house_x${REPS}.log
      ros1_log="house ROS1 Node B n=$REPS (m=1..16, TIMEOUT=120)"
      ros1_hdr="# house 本家 value_iteration Node B n=$REPS 反復掃引: goal (6.0,-2.0,90°),
# goal_margin 0.3m/±15°, safety 0.2m/1e5, theta=60, delta_threshold=$DELTA_THR (=0.1s), TIMEOUT=120"
      ;;
    lite)
      map=vi_compare/results/tsudanuma/lite/map_tsudanuma_lite.yaml
      gx=57.375 gy=66.075 gt=0 solver=frontier2d_sparse virs_mlist=""
      virs_out=vi_compare/results/tsudanuma/sweep_vi_rs_sparse_lite_x${REPS}.csv
      virs_log="lite vi_rs sparse n=$REPS (m=1..16)"
      virs_hdr="# 津田沼 lite (540x540x60, 0.15m, 自由 9.73M = 論文 Actual の 98.2%) frontier2d_sparse n=$REPS 反復掃引
# goal (57.375,66.075,0°), 0.3m/±15°, 安全0.2m/1e5, unknown=obstacle, 18bit u64。VI_THREADS=m"
      ros1_reps=1 ros1_timeout=900 ros1_mlist=""
      ros1_csv=vi_compare/results/tsudanuma/sweep_ros1.csv
      ros1_outdir=/results/tsudanuma/lite
      ros1_runlog=/workspace/vi_compare/results/tsudanuma/lite/run_lite_sweep.log
      ros1_log="lite ROS1 Node B 単発掃引 (m=1..16, TIMEOUT=900 → 約 2 h)"
      ;;
    fullext)
      map=vi_compare/results/tsudanuma/fullext/map_tsudanuma_fullext.yaml
      gx=164.175 gy=125.625 gt=0 solver=frontier2d_sparse virs_mlist=""
      virs_out=vi_compare/results/tsudanuma/sweep_vi_rs_sparse_fullext_x${REPS}.csv
      virs_log="fullext vi_rs sparse n=$REPS (m=1..16, 各 run 状態 157M 確保 → 約 70 min)"
      virs_hdr="# 津田沼 full-extent (論文条件一致: 157,118,520 総状態 / 自由 9,727,800) frontier2d_sparse n=$REPS 反復掃引
# goal (164.175,125.625,0°) = lite goal の full 座標, 0.3m/±15°, 安全0.2m/1e5, unknown=obstacle, 18bit u64"
      ros1_reps=1 ros1_timeout=900 ros1_mlist="8 16"
      ros1_csv=vi_compare/results/tsudanuma/sweep_ros1_fullext.csv
      ros1_outdir=/results/tsudanuma/fullext
      ros1_runlog=/workspace/vi_compare/results/tsudanuma/fullext/run_fullext.log
      ros1_log="fullext ROS1 Node B (m=8,16, TIMEOUT=900 → 約 33 min)"
      ;;
    tsukuba)
      map=vi_compare/results/tsukuba/map_tsukuba_pooled.yaml
      gx=20.5 gy=-1.0 gt=0 solver=frontier2d_sparse_compact virs_mlist="4 8 12 16"
      virs_out=vi_compare/results/tsukuba/sweep_vi_rs_compact_015_x${REPS}.csv
      virs_log="tsukuba vi_rs sparse_compact n=$REPS (m=4,8,12,16 → 約 80 min)"
      virs_hdr="# tsukuba 0.15m (4417x2367x60 = 総 627M / 自由 71.3M) frontier2d_sparse_compact n=$REPS 反復掃引
# goal (20.5,-1.0,0°), 0.3m/±15°, 安全0.2m/1e5, unknown=obstacle, 18bit u64, band=auto, RAM sink
# plain sparse は常駐 ~59GB で WSL 54GiB に載らないため compact（収束値 bit-exact）を採用"
      ;;
    fullassets)
      map=vi_compare/results/tsudanuma/full/map_tsudanuma_015.yaml
      gx=179.9 gy=43.7 gt=0 solver=frontier2d_sparse virs_mlist="4 8 12 16"
      virs_out=vi_compare/results/tsudanuma/sweep_vi_rs_sparse_full_x${REPS}.csv
      virs_log="full(assets) vi_rs sparse n=$REPS (m=4,8,12,16・参考行)"
      virs_hdr="# 津田沼 full(assets) (1963x1334x60, 自由 44M = 黒線なし・論文の 4.4×・参考) frontier2d_sparse n=$REPS
# goal (179.9,43.7,0°), 0.3m/±15°, 安全0.2m/1e5, unknown=obstacle, 18bit u64"
      ros1_reps=1 ros1_timeout=900 ros1_mlist="16"
      ros1_csv=vi_compare/results/tsudanuma/sweep_ros1_full.csv
      ros1_outdir=/results/tsudanuma/full
      ros1_runlog=/workspace/vi_compare/results/tsudanuma/full/run_fullassets.log
      ros1_log="full(assets) ROS1 Node B (m=16, TIMEOUT=900・参考行)"
      ;;
  esac
  if [ "$t" = "fullext" ]; then preflight_fullext_map; fi

  if [ "${ROS1_ONLY:-0}" != "1" ]; then
    preflight_virs
    bak "$virs_out"
    log "$virs_log"
    MAP="$map" OUT="$virs_out" GOAL_X=$gx GOAL_Y=$gy GOAL_THETA=$gt \
      SOLVER=$solver MLIST="$virs_mlist" REPS="$REPS" \
      bash "$REPS_VIRS"
    prepend_hdr "$virs_out" <<<"$virs_hdr"
    python3 "$AGG" "$virs_out"
  fi
  if [ "${VIRS_ONLY:-0}" = "1" ]; then return 0; fi
  preflight_ros1
  if [ "$t" = "tsukuba" ]; then
    # 本家は states 構築で常駐 ~30GB＋一時 ~50GB（627M states, vector doubling）
    mem_guard 50000 "tsukuba ROS1 states 構築"
    log "tsukuba ROS1 Node B (m=16, TIMEOUT=900, 構築 ~190s 込み → 約 19 min)"
    mkdir -p vi_compare/results/tsukuba/snap_run
    docker_ros1 vi_bench_ros1_tsukuba "$SNAP_TSUKUBA" \
      /workspace/vi_compare/results/tsukuba/snap_run/ros1_015_g15_t900_run.log \
      MAP_YAML=/workspace/vi_compare/results/tsukuba/map_tsukuba_pooled.yaml \
      GOAL_X=20.5 GOAL_Y=-1.0 GOAL_YAW=0 \
      GOAL_MARGIN_RADIUS=0.3 GOAL_MARGIN_THETA=15 \
      DELTA_THR=$DELTA_THR TIMEOUT=900 THREAD_NUM=16 \
      VI_SNAP_MS=10000000 OUT_PREFIX=ros1_015_g15_m16_t900
    python3 -c "import json;j=json.load(open('vi_compare/results/tsukuba/snap_run/ros1_015_g15_m16_t900.json'));print({k:j.get(k) for k in ('elapsed_sec','sweeps','converged','last_max_delta','thread_num')})"
    return 0
  fi
  bak "$ros1_csv"
  log "$ros1_log"
  docker_ros1 "vi_bench_ros1_$t" "$REPS_ROS1" "$ros1_runlog" \
    MAP_YAML="/workspace/$map" GOAL_X=$gx GOAL_Y=$gy GOAL_YAW=$gt \
    DELTA_THR=$DELTA_THR TIMEOUT=$ros1_timeout MLIST="$ros1_mlist" REPS=$ros1_reps \
    OUTDIR="$ros1_outdir" SWEEP_CSV="/results/${ros1_csv#vi_compare/results/}"
  if [ -n "$ros1_hdr" ]; then prepend_hdr "$ros1_csv" <<<"$ros1_hdr"; fi
  python3 "$AGG" "$ros1_csv"
}

# ------------------------------------------------------------------ main ----
targets=("$@")
[ ${#targets[@]} -eq 0 ] && targets=(all)
for t in "${targets[@]}"; do
  case "$t" in
    all) bench house; bench lite; bench fullext; bench tsukuba ;;
    house|lite|fullext|tsukuba|fullassets) bench "$t" ;;
    *) echo "unknown target: $t (house|lite|fullext|tsukuba|fullassets|all)" >&2; exit 1 ;;
  esac
done
log "全ベンチ完了"
