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
(vi_global_planner/launch/navigation_launch.py does this).

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
            parameters=[{
                'use_sim_time': use_sim_time,
                'solver': solver,
                'pose_topic': pose_topic,
                'scan_topic': scan_topic,
                # ── 価値関数の定義 (広域・狭域で共有) ──
                'theta_cell_num': 60,        # 360 を割り切る値であること (t_resolution が整数除算)
                'safety_radius': 0.2,
                'safety_radius_penalty': 30,
                'goal_margin_radius': 0.3,   # [m] final_state = ゴール許容差
                'goal_margin_theta': 15.0,   # [deg]
                'map_wait_sec': 60,
                'vi_threads': 0,             # 0 = 論理コア数 (VI_THREADS)
                'solve_chunk': 64,           # cancel 観測間隔 (イテレーション)
                'max_solve_iter': 1000000,
                'goal_tolerance_xy': 0.25,   # [m] 価値関数キャッシュ再利用の許容ゴール差
                'goal_tolerance_deg': 10.0,  # [deg]
                'unknown_as_obstacle': True,
                # ── 広域 (compute_path_to_pose) ──
                'max_rollout_steps': 10000,
                'start_tolerance': 0.5,      # [m] start が方策なしセルのときの近傍探索半径
                'path_spacing': 0.05,        # [m] 経路補間間隔 (0 で無効)
                # ── 狭域 (follow_path) ──
                'control_frequency': 10.0,   # [Hz] 本家 ViNode::decision の 100ms
                'refine_budget_ms': 40,      # [ms/tick] ウィンドウ価値反復の時間予算
                'action_tolerance': 0.2,     # [m] 方策なしセルでの近傍行動探索半径
                'no_action_timeout_sec': 3.0,
                # ── 可視化 ──
                'publish_value_function': True,
                'value_publish_interval_ms': 500,
                'cost_drawing_threshold': 60,         # value_function のスケール上限
                'window_cost_drawing_threshold': 60,  # local_window_value のスケール上限
                # ── 行動集合 (ここで決めた値がそのまま効く) ──
                # 遷移表は起動時にこの値から作られるので、名前も歩幅も回転量も
                # 好きに変えてよい (数も 6 でなくてよい)。3 本の配列は同じ長さで
                # あること。既定は本家 launch と同じ 6 行動。
                'action_names': ['forward', 'back', 'right', 'rightfw', 'left', 'leftfw'],
                'action_forward_m': [0.3, -0.2, 0.0, 0.2, 0.0, 0.2],
                'action_rotation_deg': [0.0, 0.0, -20.0, -20.0, 20.0, 20.0],
            }],
        ),
    ])
