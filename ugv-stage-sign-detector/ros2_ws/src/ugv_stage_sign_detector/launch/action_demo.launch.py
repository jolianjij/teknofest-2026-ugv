"""Launch the generic dispatcher with a safe simulated action server."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="ugv_stage_sign_detector",
                executable="mock_stage_action_server",
                name="mock_stage_action_server",
                output="screen",
            ),
            Node(
                package="ugv_stage_sign_detector",
                executable="stage_action_dispatcher_node",
                name="stage_action_dispatcher",
                output="screen",
            ),
        ]
    )
