"""Standalone launch for vi_local_planner (Nav2 controller_server replacement).

Expects /map (transient_local, e.g. nav2_map_server), a
PoseWithCovarianceStamped pose topic (emcl2: mcl_pose, AMCL: amcl_pose) and a
sensor_msgs/LaserScan topic to be provided by the surrounding bringup.
cmd_vel is published as-is here; in a full Nav2 bringup remap it to
cmd_vel_nav so it passes through velocity_smoother (see
vi_global_planner/launch/navigation_launch.py with local_planner:=vi).
"""
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
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('solver', default_value='frontier2d_sparse'),
        Node(
            package='vi_local_planner',
            executable='vi_local_planner',
            name='vi_local_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'solver': solver,
                'pose_topic': pose_topic,
                'scan_topic': scan_topic,
                'theta_cell_num': 60,
                'safety_radius': 0.2,
                'safety_radius_penalty': 30,
                'goal_margin_radius': 0.3,
                'goal_margin_theta': 15.0,
                'map_wait_sec': 60,
                'vi_threads': 0,             # 0 = 論理コア数 (VI_THREADS)
                'solve_chunk': 64,           # cancel 観測間隔 (イテレーション)
                'goal_tolerance_xy': 0.25,   # [m] 価値関数キャッシュ再利用の許容ゴール差
                'goal_tolerance_deg': 10.0,  # [deg]
                'control_frequency': 10.0,   # [Hz] 本家 ViNode::decision の 100ms
                'refine_budget_ms': 40,      # [ms/tick] ローカル価値反復の時間予算
                'action_tolerance': 0.2,     # [m] 方策なしセルでの近傍行動探索半径
                'no_action_timeout_sec': 3.0,
                'unknown_as_obstacle': True,
                'action_names': ['forward', 'back', 'right', 'rightfw', 'left', 'leftfw'],
                'action_forward_m': [0.3, -0.2, 0.0, 0.2, 0.0, 0.2],
                'action_rotation_deg': [0.0, 0.0, -20.0, -20.0, 20.0, 20.0],
            }],
        ),
    ])
