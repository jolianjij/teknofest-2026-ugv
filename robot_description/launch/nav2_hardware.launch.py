"""
Real-hardware counterpart to nav2_sim.launch.py (2026-09-11) - same Nav2
node list and design (no map_server/AMCL, MPPI controller, reactive-only
costmaps in the "odom" frame - see nav2_params_hardware.yaml's header
comment), but nothing Gazebo-specific: no gz_sim/spawn_robot/gz_bridge, and
robot_state_publisher runs with use_sim_time: false against the real clock.

Odometry is the one genuinely new piece versus the sim version. There are
no wheel/track encoders on the real vehicle (see CLAUDE.md Open Decisions
"Encoders"), so this launches rf2o_laser_odometry - 2D frame-to-frame laser
scan matching, no map required, matching this whole stack's deliberate
no-map design - to produce the odom0 input ekf_hardware.yaml fuses with the
BNO055 IMU (already flowing on /esp32/imu via serial_bridge.py).

UNVALIDATED, in more ways than usual for this project:
  - rf2o_laser_odometry has never been run against this lidar's real scan
    data - if its estimate proves too noisy/drifty on the actual course
    surface (thin/reflective obstacles, fast rotation, etc.), that is a
    real risk with laser-only odometry and would need revisiting (e.g.
    fusing IMU-only with no position estimate, accepting Nav2 can't hold a
    position at all, rather than trusting a bad one).
  - The exact parameter names below match the common ROS2 port of
    rf2o_laser_odometry as of this writing - confirm against whatever
    version actually installs on the Pi before trusting this file; a
    version/param mismatch would show up immediately as the node failing
    to start or /rf2o_laser_odometry/odom never publishing, not as a
    subtle behavior difference.
  - This has never been launched together with mission_manager.py's own
    Phase 1 reactive corridor-following (see that file's docstring) - both
    publish /cmd_vel and neither currently defers to the other. Do NOT run
    both at once until that is explicitly decided; running this launch
    file replaces mission_manager.py's role, it does not complement it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'teknofest.urdf.xacro')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_hardware.yaml')
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params_hardware.yaml')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    # Laser-scan-matching odometry (see module docstring) - publish_tf=false
    # since the EKF below (publish_tf: true in ekf_hardware.yaml) is what
    # should own the odom->base_footprint transform; two nodes publishing
    # the same TF is a real, easy-to-miss bug, not a redundancy.
    rf2o_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/rf2o_laser_odometry/odom',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
            'use_sim_time': False,
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odometry/filtered')],
    )

    # Same deliberate exact-node-list choice as nav2_sim.launch.py (not
    # nav2_bringup's navigation_launch.py) - see that file's own comment.
    # lifecycle_nodes must stay in sync with nav2_nodes below.
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
             output='screen', parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': lifecycle_nodes}]),
    ]

    return LaunchDescription([
        robot_state_publisher,
        rf2o_odometry,
        ekf_node,
        *nav2_nodes,
    ])
