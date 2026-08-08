"""Standalone launch for vi_planner.

One node serving BOTH Nav2 actions from a single value function:
`compute_path_to_pose` (replaces nav2_planner's planner_server) and
`follow_path` (replaces nav2_controller's controller_server). A goal is
therefore solved once, not once per node as it was with vi_global_planner and
vi_local_planner side by side.

Expects /map (transient_local, e.g. nav2_map_server), a
PoseWithCovarianceStamped pose topic (emcl2: mcl_pose, AMCL: amcl_pose) and a
LaserScan topic to be provided by the surrounding bringup. In a full Nav2
bringup, remap cmd_vel to cmd_vel_nav so velocity_smoother stays in the loop
(launch/navigation_launch.py does this).

Both actions read the same ValueIterator::states, so the laser penalties the
follow loop injects are visible to the global rollout -- but only once they are
propagated. The windowed refinement stops at the +-1m window, so global_sweep
(on by default) sweeps the whole shared field in the background, in chunks that
yield the lock so the 10Hz follow loop keeps running. Without it the robot
avoids obstacles locally while compute_path_to_pose keeps returning a path
through them.

map_scale and the out-of-core (compact) solver are both supported, and
global_sweep works under both. A compact solve never allocates states, so the
follow loop hydrates a patch around the robot from the sink and writes the
window back every tick -- that makes the sink itself the shared field -- and the
propagation repairs the sink one tile at a time (interior plus a frozen halo)
instead of sweeping a full states array. Same update rule, so it settles on the
same fixed point, and the work scales with the region the change actually
reaches rather than with the map. Dense costs 80 B/state (56 for states plus 24
for sweep_orders) and dense_limit_mb refuses to start above the limit; compact
keeps 12 B/state in the sink plus one follow patch and one repair tile (a few MB
together).
compact_ram_limit_mb (default 512 MB) spills the sink to disk when
compact_sink_dir is unset and the finalized output would not fit -- but the
follow loop re-reads the sink on every patch recenter, so prefer pointing
compact_sink_dir at the fastest real disk rather than leaning on that fallback.
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
