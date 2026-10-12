import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'teknofest.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'test_track.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'teknofest_robot', '-z', '0.3'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odometry/filtered')],
    )

    # Instantiating each node directly (not nav2_bringup's navigation_launch.py)
    # deliberately - that bundled launch also pulls in smoother_server,
    # route_server, and docking_server, none of which we configured or want
    # (no docking station exists, docking_server failed lifecycle configure
    # with no usable defaults). This list is exactly what we need and nothing
    # else, and the lifecycle_manager's node_names below must stay in sync
    # with it.
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor',
    ]

    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             output='screen', parameters=[nav2_config]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': lifecycle_nodes}]),
    ]

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        ekf_node,
        *nav2_nodes,
    ])
