# Nav2 navigation bringup with value-iteration planning.
#
# Robot-agnostic counterpart of nav2_bringup/launch/navigation_launch.py
# (Humble, Copyright (c) 2018 Intel Corporation, Apache License 2.0), with
# these changes:
#
#   * nav2_planner's planner_server is NOT launched and is removed from the
#     lifecycle manager's node list.
#   * `local_planner` selects between two mutually exclusive VI wirings. Both
#     are plain non-lifecycle processes (rclrs has no lifecycle support) and
#     run identically in either composition mode:
#
#       local_planner:=vi (default)
#         vi_planner alone. One node, one value function, both actions
#         (compute_path_to_pose + follow_path). controller_server is not
#         launched and is dropped from the lifecycle list. A goal is solved
#         once — the earlier vi_global_planner + vi_local_planner pair solved
#         the same value function twice, in two processes.
#
#       local_planner:=nav2
#         vi_global_planner (compute_path_to_pose only) + nav2_controller's
#         controller_server (DWB etc.). This is the wiring to use for maps that
#         need the out-of-core solver or map_scale, which vi_planner does not
#         support.
#
#     Exactly one of the two serves compute_path_to_pose — never launch both,
#     or bt_navigator binds to whichever action server it happens to discover.
#
# Any Nav2 robot can include this file in place of nav2_bringup's
# navigation_launch.py to switch to value-iteration planning. Both nodes read
# the robot pose from a PoseWithCovarianceStamped topic (`pose_topic` launch
# argument; emcl2: mcl_pose / AMCL: amcl_pose) because rclrs has no tf2 binding
# yet. Parameters may be supplied via `vi_planner:` / `vi_global_planner:`
# sections in `params_file`; anything unset falls back to the node's built-in
# defaults.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    pose_topic = LaunchConfiguration('pose_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    local_planner = LaunchConfiguration('local_planner')

    # local_planner:=vi のとき、vi_global_planner と controller_server の代わりに
    # 両アクションを持つ vi_planner (非 lifecycle) を 1 つだけ起動する。
    use_vi_local = PythonExpression(["'", local_planner, "' == 'vi'"])

    # planner_server を除いた lifecycle 管理リスト。
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']
    # local_planner:=vi ではさらに controller_server も抜く。
    lifecycle_nodes_vi_local = [n for n in lifecycle_nodes if n != 'controller_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_pose_topic_cmd = DeclareLaunchArgument(
        'pose_topic', default_value='mcl_pose',
        description='PoseWithCovarianceStamped topic vi_planner / vi_global_planner use as '
                    'the robot pose (emcl2: mcl_pose / AMCL: amcl_pose)')

    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic', default_value='scan',
        description='LaserScan topic vi_planner uses for local penalties '
                    '(only with local_planner:=vi)')

    declare_local_planner_cmd = DeclareLaunchArgument(
        'local_planner', default_value='vi',
        description="Local planner: 'vi' (default) = vi_planner, one node serving "
                    "both compute_path_to_pose and follow_path from a single value "
                    "function; 'nav2' = vi_global_planner (global only) plus "
                    "nav2_controller's controller_server — the wiring to use when "
                    "the map needs the out-of-core solver or map_scale")

    # local_planner:=vi: vi_planner が compute_path_to_pose と follow_path の両方を
    # 提供するので、vi_global_planner も controller_server も起動しない。Rust ノード
    # (非 composable・非 lifecycle) なので composition の有無によらず単独プロセス。
    # cmd_vel は controller_server と同じく velocity_smoother 経由 (cmd_vel_nav)。
    vi_planner_node = Node(
        condition=IfCondition(use_vi_local),
        package='vi_planner',
        executable='vi_planner',
        name='vi_planner',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params,
                    {'use_sim_time': use_sim_time,
                     'pose_topic': pose_topic,
                     'scan_topic': scan_topic}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')])

    # local_planner:=nav2: 広域だけ VI、追従は controller_server。vi_planner とは
    # 排他 (両方立てると compute_path_to_pose のサーバが 2 つになる)。
    vi_global_planner_node = Node(
        condition=UnlessCondition(use_vi_local),
        package='vi_global_planner',
        executable='vi_global_planner',
        name='vi_global_planner',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params,
                    {'use_sim_time': use_sim_time,
                     'pose_topic': pose_topic}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings)

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                condition=UnlessCondition(use_vi_local),
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                condition=UnlessCondition(use_vi_local),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
            Node(
                condition=IfCondition(use_vi_local),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_vi_local}]),
        ]
    )

    # ComposableNode 単体には condition を付けられないため、controller_server と
    # lifecycle_manager は local_planner の値で切り替える別 LoadComposableNodes に
    # 分離する。
    composition_and_nav2_controller = PythonExpression(
        [use_composition, " and '", local_planner, "' != 'vi'"])
    composition_and_vi_local = PythonExpression(
        [use_composition, " and '", local_planner, "' == 'vi'"])

    load_composable_controller = LoadComposableNodes(
        condition=IfCondition(composition_and_nav2_controller),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        ],
    )

    load_composable_lifecycle = LoadComposableNodes(
        condition=IfCondition(composition_and_nav2_controller),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    load_composable_lifecycle_vi_local = LoadComposableNodes(
        condition=IfCondition(composition_and_vi_local),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes_vi_local}]),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_pose_topic_cmd)
    ld.add_action(declare_scan_topic_cmd)
    ld.add_action(declare_local_planner_cmd)

    ld.add_action(vi_planner_node)
    ld.add_action(vi_global_planner_node)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_controller)
    ld.add_action(load_composable_nodes)
    ld.add_action(load_composable_lifecycle)
    ld.add_action(load_composable_lifecycle_vi_local)

    return ld
