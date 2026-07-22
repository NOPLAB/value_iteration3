"""Standalone launch for vi_global_planner (Nav2 planner_server replacement).

Expects /map (transient_local, e.g. nav2_map_server) and a
PoseWithCovarianceStamped pose topic (emcl2: mcl_pose, AMCL: amcl_pose)
to be provided by the surrounding bringup.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pose_topic = LaunchConfiguration('pose_topic')
    solver = LaunchConfiguration('solver')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'pose_topic', default_value='mcl_pose',
            description='PoseWithCovarianceStamped topic used as the robot pose '
                        '(rclrs has no tf2 yet). emcl2: mcl_pose / AMCL: amcl_pose.'),
        DeclareLaunchArgument('solver', default_value='frontier2d_sparse'),
        Node(
            package='vi_global_planner',
            executable='vi_global_planner',
            name='vi_global_planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'solver': solver,
                'pose_topic': pose_topic,
                'theta_cell_num': 60,
                'safety_radius': 0.2,
                'safety_radius_penalty': 30,
                'goal_margin_radius': 0.3,
                'goal_margin_theta': 15.0,
                'map_wait_sec': 60,
                'vi_threads': 0,          # 0 = 論理コア数 (VI_THREADS)
                'solve_chunk': 64,        # cancel 観測間隔 (イテレーション)
                'max_rollout_steps': 10000,
                'start_tolerance': 0.5,   # [m] start が方策なしセルのときの近傍探索半径
                'path_spacing': 0.05,     # [m] 経路補間間隔 (0 で無効)
                'goal_tolerance_xy': 0.25,   # [m] 価値関数キャッシュ再利用の許容ゴール差
                'goal_tolerance_deg': 10.0,  # [deg]
                'cost_drawing_threshold': 60,
                'publish_value_function': True,
                'unknown_as_obstacle': True,
                'action_names': ['forward', 'back', 'right', 'rightfw', 'left', 'leftfw'],
                'action_forward_m': [0.3, -0.2, 0.0, 0.2, 0.0, 0.2],
                'action_rotation_deg': [0.0, 0.0, -20.0, -20.0, 20.0, 20.0],
            }],
        ),
    ])
