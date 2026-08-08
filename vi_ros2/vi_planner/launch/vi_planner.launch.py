"""Standalone launch for vi_planner: one node serving both compute_path_to_pose
and follow_path from a single value function. Expects /map (transient_local), a
PoseWithCovarianceStamped pose topic and a LaserScan topic; in a full Nav2
bringup remap cmd_vel to cmd_vel_nav (launch/navigation_launch.py does this).
Details: repo CLAUDE.md, vi_ros2 section."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pose_topic = LaunchConfiguration('pose_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    solver = LaunchConfiguration('solver')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'pose_topic', default_value='mcl_pose',
            description='PoseWithCovarianceStamped topic used as the robot pose '
                        '(rclrs has no tf2 yet). emcl2: mcl_pose / AMCL: amcl_pose.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='scan',
            description='LaserScan topic used for the local penalties injected '
                        'into the +-1m window while following.'),
        DeclareLaunchArgument('solver', default_value='frontier2d_sparse'),
        Node(
            package='vi_planner',
            executable='vi_planner',
            name='vi_planner',
            output='screen',
            # 上書きしたい値だけ書く — 未指定はノード既定 (read_params の
            # default) が効く。行動集合も action_names/action_forward_m/
            # action_rotation_deg でここから変えられる (3 本は同じ長さ)。
            parameters=[{
                'use_sim_time': use_sim_time,
                'solver': solver,
                'pose_topic': pose_topic,
                'scan_topic': scan_topic,
                'map_wait_sec': 60,  # ノード既定 30 より長め (map_server 起動待ち)
            }],
        ),
    ])
