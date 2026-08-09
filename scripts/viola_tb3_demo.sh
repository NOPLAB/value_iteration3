#!/usr/bin/env bash
# VIOLA (vi_planner standalone + 内蔵の全地図 belief + QMDP) を TurtleBot3
# house (Gazebo classic) で走らせるデモ。ネイティブ RoboStack 環境用。
# WORLD=world で従来の turtlebot3_world (円柱 3 本、自由空間 19 m²) に戻せる。
#
# このデモの belief は VI と同じ格子に載る全地図フィールド (窓も多重解像度レベルも
# 無い)。窓つきの grid / adaptive も併存していて localizer で選べる — 違いは
# localizer の行を参照。
#
#   scripts/ros2_build.sh          # 先にネイティブビルド
#   scripts/viola_tb3_demo.sh      # Gazebo + map_server + vi_planner + RViz
#
# 起動後 (端末に "seeded" が出てから):
#   - belief は spawn 位置 (-2, -0.5, 0°) に自動シード済み。ずらしたいときは
#     RViz の「2D Pose Estimate」で撃ち直す (pose_topic = initialpose)。
#   - ゴールは RViz の「2D Goal Pose」で投入 — /goal_pose に出るだけなので、
#     standalone の vi_planner が直接受けて navigate_to_pose と同じ経路で
#     走らせる (nav2_rviz_plugins は不要)。CLI なら (house のリビング):
#       ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
#         '{header: {frame_id: map}, pose: {position: {x: -2.0, y: 3.0}}}'
#   - map→odom TF は vi_planner が配信 (publish_tf、AMCL の契約の置き換え)。
#     RViz のロボット・scan は推定姿勢に乗る (真値は Gazebo ウィンドウ)。
#     ピンク矢印 (viola_pose) は推定そのもの — ロボットモデルと重なるのが正常。
#   - RViz の Belief 表示 (/belief) が belief の θ 周辺分布。value_function /
#     local_window_value と同じ Map 表示・同じ間引き (value_publish_interval_ms)
#     で、質量ゼロは透過・ピークが赤。収束すると 1〜数セルまで縮むので、
#     広がって見えるのは誘拐やリセットの直後だけ。地図と重なってちらつくときは
#     Displays で Map か ValueFunction を一時的に切る。
#
# 各コンポーネントのログ: out/viola_demo_logs/*.log (無反応のときはまずここ)。
#
# house の地図は turtlebot3 に同梱されていないので scripts/gen_house_map.py で
# 生成する (model.sdf の壁 box を LDS 高さ 0.18 m で水平スライス → ドア上部の
# 垂れ壁は開口として抜ける)。412x310 @0.05 m、自由空間 308 m²。
# この設定での実測: (-2, -0.5) → (-2, 3.0) (玄関を抜けてリビング) が 44 s で
# ゴール到達 (solve 4.87 s、内蔵 belief のみで真値は一切戻していない)。
#
# 外部推定と比べたいときは scripts/vip_tb3_demo.sh (localizer=external + emcl2)。
set -euo pipefail
WORLD="${WORLD:-house}"   # house | world — launch 名と assets/tb3_$WORLD/ に対応
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$REPO_ROOT/scripts/ros2_native_env.sh"
set +u; . "$REPO_ROOT/vi_ros2_ws/install/local_setup.sh"; set -u
export TURTLEBOT3_MODEL=burger

LOG="$REPO_ROOT/out/viola_demo_logs"
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

# vi_planner の全パラメータ (main.rs の宣言順)。値はこのデモの設定で、各行の
# コメントが規定値。規定値から変えているのは use_sim_time / map_wait_sec /
# pose_topic / publish_tf / localizer / follow_controller / qmdp / standalone
# のみ。
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
    # -p compact_sink_dir:=/path         # 規定: "" = RAM (compact ソルバのときだけ効く)
    -p compact_ram_limit_mb:=512         # 規定: 512 — sink がこれを超えると /tmp/vi_planner_sink へ
    -p dense_limit_mb:=1500              # 規定: 1500 — 密の価値関数がこれを超えたら起動拒否
    -p vi_threads:=0                     # 規定: 0 = VI_THREADS を触らない (>0 で設定)
    -p max_solve_iter:=1000000           # 規定: 1000000 (solve の反復上限)
    -p solve_chunk:=64                   # 規定: 64 (キャンセル/進捗確認の粒度)
    # ── ゴール判定 / 姿勢・TF ──
    -p goal_tolerance_xy:=0.25           # 規定: 0.25 [m] (navigate_to_pose の達成判定)
    -p goal_tolerance_deg:=10.0          # 規定: 10.0 [deg]
    -p pose_topic:=initialpose           # 規定: mcl_pose — 内蔵 belief では手動シード口
    -p global_frame:=map                 # 規定: map
    -p publish_tf:=true                  # 規定: false — map→odom TF を配信 (AMCL の契約の置き換え)
    -p transform_tolerance:=0.5          # 規定: 0.5 [s] (TF スタンプの未来日付け)
    -p odom_topic:=odom                  # 規定: odom (publish_tf 用の T_odom→base の出どころ)
    # ── 自己位置推定 ──
    -p localizer:=grid                 # 規定: external。内蔵は 2 系統:
                                         #   窓つき grid | adaptive | adaptive_viterbi
                                         #     (belief_radius の窓だけを持つ。adaptive は観測が合わなく
                                         #      なると粗い広域レベルへ広げて再定位 = 誘拐から復帰でき、
                                         #      未シードでも立ち上がる。active_reloc が使えるのもこれ)
                                         #   全地図 belief (sum-product) | viterbi (同 min-plus)
                                         #     (窓もレベル機構も無く、VI と同じ格子に belief を全域で持つ)
                                         #   viterbi は observe が全域走査なので tb3 実測 183 ms/tick —
                                         #   追従ループの 40 ms 予算を超える。ベンチ用と割り切る。
    -p belief_radius:=2.0                # 規定: 2.0 [m] — 窓つき (grid/adaptive) の belief 窓の半径。
                                         #   全地図 (belief/viterbi) では使わない。
    -p belief_sensor_sigma:=0.2          # 規定: 0.2 [m] (尤度場のガウス幅)
    -p belief_beam_step:=10              # 規定: 10 (補正に使うビームの間引き、1 = 全ビーム)
    -p belief_max_range:=25.0            # 規定: 25.0 [m] (これより遠いレンジは補正に使わない)
    -p belief_motion_sigma_xy:=0.03      # 規定: 0.03 [m/tick] (predict の位置ノイズ)
    -p belief_motion_sigma_theta_deg:=2.0 # 規定: 2.0 [deg/tick]
    -p belief_z_min:=0.05                # 規定: 0.05 (ビーム尤度の床)
    -p belief_weight_skip_ratio:=0.0001  # 規定: 1e-4 (補正で読む重みの相対しきい値)
    -p belief_reset_quality:=0.25        # 規定: 0.25 — 観測一致度がこれを割ると free 一様を混ぜて再定位
    -p belief_lost_ess:=500.0            # 規定: 500.0 — belief の有効セル数がこれを超えたらロスト
                                         #   (belief_reset_quality と併せて全地図 belief/viterbi 用。
                                         #    窓つきは自前の expand/contract カスケードで復帰する)
    -p scan_quality_gate:=0.25           # 規定: 0.25 — 観測一致度がこれ未満の scan は注入を減衰、0 で無効
    -p footprint_clear_m:=0.2            # 規定: 0.2 [m] — 注入のたびに機体周囲の local_penalty を消す、0 で無効
    -p map_clear_from_scan:=true         # 規定: false — ビームが貫通したセルは地図の壁/膨張帯も反証する
                                         #   (幽霊壁に囲まれた停止を解く。自己位置がずれていると本物の壁を
                                         #    消し得るので既定は off)
    # ── 広域 (compute_path_to_pose) ──
    -p max_rollout_steps:=10000          # 規定: 10000 (経路ロールアウトの上限歩数)
    -p start_tolerance:=0.5              # 規定: 0.5 [m] (開始姿勢と地図のずれ許容)
    -p path_spacing:=0.05                # 規定: 0.05 [m] (densify の点間隔)
    # ── 狭域 (follow_path) ──
    -p local_xy_range:=1.0               # 規定: 1.0 [m] (ローカルウィンドウ半径、本家固定値)
    -p follow:=true                      # 規定: true — false で global-only (nav2_controller と組む)
    -p scan_topic:=scan                  # 規定: scan
    -p control_frequency:=10.0           # 規定: 10.0 [Hz]
    -p refine_budget_ms:=40              # 規定: 40 [ms/tick] (ローカル反復の予算)
    -p action_tolerance:=0.2             # 規定: 0.2 [m] (方策なしセルの最近傍借用半径)
    -p no_action_timeout_sec:=3.0        # 規定: 3.0 [s] (方策なし連続でこの時間 → 追従失敗)
    -p invalid_range_m:=1000000.0        # 規定: 1e6 [m] (inf/NaN レンジの差し替え値)
    -p busy_ticks_before_stop:=3         # 規定: 3 (ロック取れず連続 n tick で停止指令)
    -p patch_slack_cells:=2              # 規定: 2 (compact パッチの寸法スラック)
    -p repair_interior_cells:=16         # 規定: 16 (修復タイルの interior 一辺)
    -p follow_controller:=mppi         # 規定: greedy (本家 decision) | dwa | mppi
    -p dwa_horizon_s:=1.0                # 規定: 1.0 [s] (DWA/MPPI の前方シミュレーション)
    -p dwa_n_v:=7                        # 規定: 7 (DWA の v 候補数)
    -p dwa_n_w:=11                       # 規定: 11 (DWA の ω 候補数)
    -p dwa_lethal_penalty:=2.0           # 規定: 2.0 [PROB_BASE 単位] (margin 帯棄却、0 で無効)
    -p mppi_samples:=256                 # 規定: 256
    -p mppi_lambda:=1.0                  # 規定: 1.0 (softmax 温度)
    -p mppi_sigma_v:=0.0                 # 規定: 0.0 = 行動集合から自動
    -p mppi_sigma_w_deg:=0.0             # 規定: 0.0 = 行動集合から自動
    -p qmdp:=true                        # 規定: false — belief が多峰の tick だけ QMDP で行動選択
                                         #   (単峰の tick は follow_controller のまま)。多峰性は峰の
                                         #   数で測る — セル数だと全地図 belief では常に真になる。
    -p active_reloc:=false               # 規定: false — ロスト中に止まって待つ代わりに、仮説を判別する
                                         #   地点への多目標 VI を解いて QMDP で走る。要 localizer:=adaptive
                                         #   (判別点を出せるのはこれだけ) + 密ソルバ。
    -p reloc_timeout_sec:=30.0           # 規定: 30.0 [s] — 能動的再定位を諦めて通常の停止待ちに戻すまで
    # ── スタンドアロン (navigate_to_pose / follow_waypoints) ──
    -p standalone:=true                  # 規定: false — bt_navigator 等の代わりに自前で提供
    -p goal_retry_limit:=3               # 規定: 3 (追従失敗時の投げ直し上限、負で無制限)
    -p goal_retry_settle_sec:=3.0        # 規定: 3.0 [s] (投げ直し前の settle — 場を動かす待ち)
    -p stop_on_failure:=false            # 規定: false (巡回で 1 点失敗しても次へ)
    -p waypoint_pause_sec:=0.2           # 規定: 0.2 [s] (点に着いてから次へ向かうまでの settle)
    # ── 狭域 → 広域フィードバック (全域掃き) ──
    -p global_sweep:=true                # 規定: true (local_penalty を全域へ伝播)
    -p global_sweep_budget_ms:=20        # 規定: 20 [ms] (1 回のロックで掃く時間)
    -p global_sweep_idle_ms:=60          # 規定: 60 [ms] (ロックを手放して待つ時間、比 = CPU 取り分)
    -p global_sweep_cells_per_step:=5000 # 規定: 5000 (密経路のチャンク粒度)
    -p global_sweep_report_sec:=2.0      # 規定: 2.0 [s] (伝播中の進捗報告・再配信間隔)
    # ── ウェイポイント先読み / 早期発進 ──
    -p waypoint_prefetch:=false          # 規定: false — 次の点の価値関数を走行中に解く (メモリ 2 倍)
    -p waypoint_topic:=waypoints         # 規定: waypoints (nav_msgs/Path, transient_local)
    -p waypoint_prefetch_threads:=1      # 規定: 1 (compact 経路のみ効く)
    -p waypoint_prefetch_poll_ms:=50     # 規定: 50 [ms]
    -p early_start:=false                # 規定: false — 現在地→ゴールが繋がった時点で solve 打ち切り
    # ── 可視化 ──
    -p publish_value_function:=true      # 規定: true (value_function / local_window_value /
                                         #   belief トピック。belief は内蔵推定器のときだけ中身が出る)
    -p value_publish_interval_ms:=500    # 規定: 500 [ms] (solve 途中経過の配信間隔、0 = 完了時のみ。
                                         #   belief にも同じ間引きが掛かる — 0 では belief も出ない)
    -p cost_drawing_threshold:=60        # 規定: 60 (value_function の描画スケール上限)
    -p window_cost_drawing_threshold:=60 # 規定: 60 (local_window_value 用)
)
ros2 run vi_planner vi_planner --ros-args "${VI_ARGS[@]}" >"$LOG/vi_planner.log" 2>&1 &

rviz2 -d "$REPO_ROOT/assets/tb3_$WORLD/viola.rviz" >"$LOG/rviz.log" 2>&1 &

# map_server の lifecycle 化。GUI 起動で discovery が遅れても諦めない
# (一発勝負にすると空振り → /map が出ず vi_planner が map 待ちで死ぬ)。
for _ in $(seq 1 30); do
    if ros2 run nav2_util lifecycle_bringup map_server >>"$LOG/lifecycle.log" 2>&1; then
        echo "viola_demo: map_server active"
        break
    fi
    sleep 2
done

# belief の自動シード: vi_planner の購読が立ってから撃つ (それ以前は消える)。
for _ in $(seq 1 60); do
    # トピック未出現のうちは ros2 topic info が非ゼロ終了する (set -e に殺させない)
    n="$(ros2 topic info /initialpose 2>/dev/null | sed -n 's/Subscription count: //p' || true)"
    if [ "${n:-0}" -ge 1 ]; then
        ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
            '{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}}}}' \
            >"$LOG/seed.log" 2>&1
        echo "viola_demo: seeded belief at spawn (-2.0, -0.5) — RViz の Nav2 Goal でゴールを撃てます"
        break
    fi
    sleep 2
done

wait
