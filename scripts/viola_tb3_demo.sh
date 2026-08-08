#!/usr/bin/env bash
# VIOLA (vi_planner standalone + GridLocalizer) を TurtleBot3 world (Gazebo classic)
# で走らせるデモ。ネイティブ RoboStack 環境用。
#
#   scripts/ros2_build.sh          # 先にネイティブビルド
#   scripts/viola_tb3_demo.sh      # Gazebo + map_server + vi_planner + RViz
#
# 起動後:
#   - belief は spawn 位置 (-2, -0.5, 0°) に自動シード済み。ずらしたいときは
#     RViz の「2D Pose Estimate」で撃ち直す (pose_topic = initialpose)。
#   - ゴールは RViz の「Nav2 Goal」ツールで投入 (navigate_to_pose)。CLI なら:
#       ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
#         '{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.5}}}}'
#   - map→odom の static TF は表示専用 (spawn 位置で固定)。VIOLA 自体は TF を
#     一切使わない — 推定は scan + 自分の cmd_vel だけ。
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$REPO_ROOT/scripts/ros2_native_env.sh"
set +u; . "$REPO_ROOT/vi_ros2_ws/install/local_setup.sh"; set -u
export TURTLEBOT3_MODEL=burger

trap 'kill 0' EXIT INT TERM

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &

ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$REPO_ROOT/assets/tb3_world/map.yaml" \
    -p use_sim_time:=true &
( sleep 3 && ros2 run nav2_util lifecycle_bringup map_server ) &

# 表示専用: 真値 (odom) を map に重ねる。TB3 の diff_drive は odom をスポーンの
# ワールド座標で初期化する (実測) ので恒等でよい。
ros2 run tf2_ros static_transform_publisher \
    --frame-id map --child-frame-id odom &

ros2 run vi_planner vi_planner --ros-args \
    -p localizer:=grid \
    -p standalone:=true \
    -p use_sim_time:=true \
    -p pose_topic:=initialpose \
    -p scan_topic:=scan &

# belief を spawn 位置に自動シード (ノードの map 待ち + solve 前でも latest_pose に入る)
( sleep 8 && ros2 topic pub --once /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    '{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}}}}' ) &

rviz2 -d "$REPO_ROOT/assets/tb3_world/viola.rviz" &

wait
