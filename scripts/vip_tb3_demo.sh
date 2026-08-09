#!/usr/bin/env bash
# VIP (vi_planner standalone、自己位置推定は外部 emcl2) を TurtleBot3 house
# (Gazebo classic) で走らせるデモ。viola_tb3_demo.sh の内蔵 localizer 抜き版。
# ネイティブ RoboStack 環境用。WORLD=world で従来の turtlebot3_world に戻せる。
#
#   scripts/ros2_build.sh          # 先にネイティブビルド (vi_planner)
#   # emcl2 は vi_ros2_ws/src/emcl2_ros2 を同じ ws で一度 colcon ビルドしておく:
#   #   colcon build --merge-install --packages-select emcl2 ...
#   scripts/vip_tb3_demo.sh        # Gazebo + map_server + emcl2 + vi_planner + RViz
#
# 起動後:
#   - emcl2 は initial_pose パラメータ (quick_start 設定 = spawn 位置 (-2, -0.5, 0°))
#     で自己シード済み。ずらしたいときは RViz の「2D Pose Estimate」(/initialpose
#     は emcl2 が受けてパーティクルを撒き直す)。
#   - ゴールは RViz の「2D Goal Pose」で投入 — /goal_pose に出るだけなので、
#     standalone の vi_planner が直接受けて navigate_to_pose と同じ経路で
#     走らせる (nav2_rviz_plugins は不要)。CLI なら (house のリビング):
#       ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
#         '{header: {frame_id: map}, pose: {position: {x: -2.0, y: 3.0}}}'
#   - map→odom TF は emcl2 が配信 (AMCL の契約)。vi_planner は mcl_pose を
#     購読するだけ (localizer=external、publish_tf=false)。
#
# 各コンポーネントのログ: out/vip_demo_logs/*.log (無反応のときはまずここ)。
#
# 内蔵推定版 (scripts/viola_tb3_demo.sh) との比較用。
set -euo pipefail
WORLD="${WORLD:-house}"   # house | world — launch 名と assets/tb3_$WORLD/ に対応
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$REPO_ROOT/scripts/ros2_native_env.sh"
set +u; . "$REPO_ROOT/vi_ros2_ws/install/local_setup.sh"; set -u
export TURTLEBOT3_MODEL=burger

LOG="$REPO_ROOT/out/vip_demo_logs"
mkdir -p "$LOG"

# 前回の残骸 gzserver/gzclient が居ると新しい gzserver が即死する (exit 255)。
# Gazebo classic は SIGTERM で綺麗に死なないことがあるので起動前に掃除する。
pkill -x gzserver 2>/dev/null && sleep 2 || true
pkill -x gzclient 2>/dev/null || true

# kill 0 は自分にも届くので、再帰しないよう先にトラップを解除する
trap 'trap - EXIT INT TERM; kill 0' EXIT INT TERM

ros2 launch turtlebot3_gazebo "turtlebot3_$WORLD.launch.py" >"$LOG/gazebo.log" 2>&1 &

ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$REPO_ROOT/assets/tb3_$WORLD/map.yaml" \
    -p use_sim_time:=true >"$LOG/map_server.log" 2>&1 &

# 外部自己位置推定: emcl2 (expansion resetting MCL)。quick_start 設定は
# initial_pose = TB3 の spawn 位置 (house/world 共通) (-2, -0.5, 0°) 込み。/map を
# transient_local で購読し、mcl_pose と map→odom TF を配信する。
ros2 run emcl2 emcl2_node --ros-args \
    --params-file "$REPO_ROOT/vi_ros2_ws/install/share/emcl2/config/emcl2_quick_start.param.yaml" \
    -p use_sim_time:=true >"$LOG/emcl2.log" 2>&1 &

# vi_planner の全パラメータ (main.rs の宣言順)。値はこのデモの設定で、各行の
# コメントが規定値。規定値から変えているのは use_sim_time / map_wait_sec /
# follow_controller / standalone のみ。内蔵 belief 系のパラメータ
# (belief_* / qmdp) は localizer=external では未使用なので省略 — 内蔵推定を
# 使う構成は viola_tb3_demo.sh 参照。値反復の belief 次元 (belief_levels) も
# 内蔵推定が b̂ を供給しないと使えないので、ここでは規定の 1 (= 3D) のまま。
VI_ARGS=(
    -p use_sim_time:=true                # 規定: false (ROS 標準) — Gazebo の /clock に乗る
    # ── ソルバ / 地図 ──
    -p solver:=frontier2d_sparse         # 規定: frontier2d_sparse (U64Solver の名前)
    -p theta_cell_num:=60                # 規定: 60 (θ の離散数)
    -p safety_radius:=0.2                # 規定: 0.2 [m] (障害物マージン帯の半径)
    -p safety_radius_penalty:=30         # 規定: 30 [秒相当/セル] (マージン帯の penalty)
    -p goal_margin_radius:=0.3           # 規定: 0.3 [m] (ゴール判定の XY 半径)
    -p goal_margin_theta:=15.0           # 規定: 15.0 [deg] (ゴール判定の θ 幅)
    -p map_wait_sec:=120                 # 規定: 30 — GUI 起動で discovery が遅れるので延長
    # -p action_names:="[forward, back, right, rightfw, left, leftfw]"   # 規定: 左記 6 行動
    # -p action_forward_m:="[0.3, -0.2, 0.0, 0.2, 0.0, 0.2]"             # 規定: 左記 [m]
    # -p action_rotation_deg:="[0.0, 0.0, -20.0, -20.0, 20.0, 20.0]"     # 規定: 左記 [deg]
    -p unknown_as_obstacle:=true         # 規定: true (未知セルを障害物扱い)
    -p map_scale:=1                      # 規定: 1 (ダウンサンプル倍率、1 = 等倍)
    -p downsample_policy:=conservative   # 規定: conservative (障害物優先) | optimistic
    # -p compact_sink_dir:=/path         # 規定: "" = 空きメモリ次第で RAM / ディスク自動
    -p compact_ram_limit_mb:=0           # 規定: 0 = 空きメモリの半分まで RAM sink、超えたら
                                         #   /tmp/vi_planner_sink へ。>0 で MB を明示
    -p vi_threads:=0                     # 規定: 0 = VI_THREADS を触らない (>0 で設定)
    # ── ゴール判定 / 姿勢・TF ──
    -p goal_tolerance_xy:=0.25           # 規定: 0.25 [m] (navigate_to_pose の達成判定)
    -p goal_tolerance_deg:=10.0          # 規定: 10.0 [deg]
    -p pose_topic:=mcl_pose              # 規定: mcl_pose — emcl2 の推定出力をそのまま使う
    -p global_frame:=map                 # 規定: map
    -p publish_tf:=false                 # 規定: false — map→odom TF は emcl2 が配信 (併用禁止)
    -p transform_tolerance:=0.5          # 規定: 0.5 [s] (publish_tf 用、ここでは未使用)
    -p odom_topic:=odom                  # 規定: odom (publish_tf 用、ここでは未使用)
    # ── 自己位置推定 ──
    -p localizer:=external               # 規定: external — pose_topic の外部推定 (emcl2) に乗る。
                                         #   内蔵は belief (全地図 sum-product) / viterbi (同 min-plus)
    -p scan_quality_gate:=0.25           # 規定: 0.25 — external は quality 1.0 固定なので実質無効
    # ── 広域 (compute_path_to_pose) ──
    -p max_rollout_steps:=10000          # 規定: 10000 (経路ロールアウトの上限歩数)
    -p path_spacing:=0.05               # 規定: 0.05 [m] (densify の点間隔)
    # ── 狭域 (follow_path) ──
    -p local_xy_range:=1.0               # 規定: 1.0 [m] (ローカルウィンドウ半径、本家固定値)
    -p follow:=true                      # 規定: true — false で global-only (nav2_controller と組む)
    -p scan_topic:=scan                  # 規定: scan
    -p control_frequency:=10.0           # 規定: 10.0 [Hz]
    -p refine_budget_ms:=40              # 規定: 40 [ms/tick] (ローカル反復の予算)
    -p no_action_timeout_sec:=3.0        # 規定: 3.0 [s] (方策なし連続でこの時間 → 追従失敗)
    -p follow_controller:=mppi           # 規定: greedy (本家 decision) | dwa | mppi
    -p dwa_horizon_s:=1.0                # 規定: 1.0 [s] (DWA/MPPI の前方シミュレーション)
    -p dwa_lethal_penalty:=2.0           # 規定: 2.0 [PROB_BASE 単位] (margin 帯棄却、0 で無効)
    -p mppi_samples:=256                 # 規定: 256 (候補数・温度・ノイズ σ はコード側で固定)
    # ── スタンドアロン (navigate_to_pose / follow_waypoints) ──
    -p standalone:=true                  # 規定: false — bt_navigator 等の代わりに自前で提供
    -p goal_retry_limit:=3               # 規定: 3 (追従失敗時の投げ直し上限、負で無制限)
    -p stop_on_failadaptive=false            # 規定: false (巡回で 1 点失敗しても次へ)
    -p waypoint_pause_sec:=0.2           # 規定: 0.2 [s] (点に着いてから次へ向かうまでの settle)
    # ── 狭域 → 広域フィードバック (全域掃き) ──
    -p global_sweep:=true                # 規定: true (local_penalty を全域へ伝播)
    -p global_sweep_duty:=25             # 規定: 25 [%] (掃きに渡す CPU の割合、100 で掃きっぱなし)
    # ── ウェイポイント先読み / 早期発進 ──
    -p waypoint_prefetch:=false          # 規定: false — 次の点の価値関数を走行中に解く (メモリ 2 倍)
    -p waypoint_topic:=waypoints         # 規定: waypoints (nav_msgs/Path, transient_local)
    -p waypoint_prefetch_threads:=1      # 規定: 1 (compact 経路のみ効く)
    -p early_start:=false                # 規定: false — 現在地→ゴールが繋がった時点で solve 打ち切り
    # ── 可視化 ──
    -p value_publish_interval_ms:=500    # 規定: 500 [ms] — value_function / local_window_value /
                                         #   belief の配信間隔。0 = solve 完了時のみ、負で可視化を立てない
    -p cost_drawing_threshold:=60        # 規定: 60 (描画スケール上限、全域スライスと窓で共通)
)
ros2 run vi_planner vi_planner --ros-args "${VI_ARGS[@]}" >"$LOG/vi_planner.log" 2>&1 &

rviz2 -d "$REPO_ROOT/assets/tb3_$WORLD/viola.rviz" >"$LOG/rviz.log" 2>&1 &

# map_server の lifecycle 化。GUI 起動で discovery が遅れても諦めない
# (一発勝負にすると空振り → /map が出ず vi_planner が map 待ちで死ぬ)。
for _ in $(seq 1 30); do
    if ros2 run nav2_util lifecycle_bringup map_server >>"$LOG/lifecycle.log" 2>&1; then
        echo "vip_demo: map_server active — emcl2 は spawn 位置に自己シード済み、RViz の 2D Goal Pose でゴールを撃てます"
        break
    fi
    sleep 2
done

wait
