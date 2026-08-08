#!/usr/bin/env bash
# VIOLA (vi_planner standalone + GridLocalizer) を TurtleBot3 world (Gazebo classic)
# で走らせるデモ。ネイティブ RoboStack 環境用。
#
#   scripts/ros2_build.sh          # 先にネイティブビルド
#   scripts/viola_tb3_demo.sh      # Gazebo + map_server + vi_planner + RViz
#
# 起動後 (端末に "seeded" が出てから):
#   - belief は spawn 位置 (-2, -0.5, 0°) に自動シード済み。ずらしたいときは
#     RViz の「2D Pose Estimate」で撃ち直す (pose_topic = initialpose)。
#   - ゴールは RViz の「2D Goal Pose」で投入 — /goal_pose に出るだけなので、
#     standalone の vi_planner が直接受けて navigate_to_pose と同じ経路で
#     走らせる (nav2_rviz_plugins は不要)。CLI なら:
#       ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
#         '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.5}}}'
#   - map→odom の static TF は表示専用 (TB3 の diff_drive は odom をスポーンの
#     ワールド座標で初期化するので恒等)。VIOLA 自体は TF を一切使わない —
#     推定は scan + 自分の cmd_vel だけ。
#
# 各コンポーネントのログ: out/viola_demo_logs/*.log (無反応のときはまずここ)。
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$REPO_ROOT/scripts/ros2_native_env.sh"
set +u; . "$REPO_ROOT/vi_ros2_ws/install/local_setup.sh"; set -u
export TURTLEBOT3_MODEL=burger

LOG="$REPO_ROOT/out/viola_demo_logs"
mkdir -p "$LOG"

# kill 0 は自分にも届くので、再帰しないよう先にトラップを解除する
trap 'trap - EXIT INT TERM; kill 0' EXIT INT TERM

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py >"$LOG/gazebo.log" 2>&1 &

ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$REPO_ROOT/assets/tb3_world/map.yaml" \
    -p use_sim_time:=true >"$LOG/map_server.log" 2>&1 &

# 表示専用: 真値 (odom) を map に重ねる。
ros2 run tf2_ros static_transform_publisher \
    --frame-id map --child-frame-id odom >"$LOG/static_tf.log" 2>&1 &

ros2 run vi_planner vi_planner --ros-args \
    -p localizer:=grid \
    -p standalone:=true \
    -p use_sim_time:=true \
    -p map_wait_sec:=120 \
    -p pose_topic:=initialpose \
    -p scan_topic:=scan >"$LOG/vi_planner.log" 2>&1 &

rviz2 -d "$REPO_ROOT/assets/tb3_world/viola.rviz" >"$LOG/rviz.log" 2>&1 &

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
