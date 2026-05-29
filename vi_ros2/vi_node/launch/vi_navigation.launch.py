"""Launch file for vi_node — default configuration matches value_iteration ROS1."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vi_node',
            executable='vi_node',
            name='vi_node',
            output='screen',
            parameters=[{
                'solver': 'frontier3d',
                'theta_cell_num': 60,
                'safety_radius': 0.2,
                'safety_radius_penalty': 30,
                'goal_margin_radius': 0.3,
                'goal_margin_theta': 15.0,
                'online': False,
                'cost_drawing_threshold': 60,
                'delta_threshold': 0,
                'thread_num': 0,
                'map_wait_sec': 30,
                'allow_action_mismatch': False,
                'action_names': ['forward','back','right','rightfw','left','leftfw'],
                'action_forward_m':  [ 0.3, -0.2,  0.0,  0.2,  0.0,  0.2],
                'action_rotation_deg': [ 0.0,  0.0, -20.0, -20.0, 20.0, 20.0],
            }],
        ),
    ])
