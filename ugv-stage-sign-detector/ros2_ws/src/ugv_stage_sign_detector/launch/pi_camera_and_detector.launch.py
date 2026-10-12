from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ml_config = PathJoinSubstitution(
        [FindPackageShare("ugv_stage_sign_detector"), "config", "detector_ml.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",
                parameters=[{"width": 1280, "height": 720}],
            ),
            Node(
                package="ugv_stage_sign_detector",
                executable="stage_sign_detector_node",
                name="stage_sign_detector",
                output="screen",
                parameters=[
                    {
                        "config_path": ml_config,
                        "image_topic": "/camera/image_raw",
                        "publish_debug_image": True,
                    }
                ],
            ),
            Node(
                package="ugv_stage_sign_detector",
                executable="stage_manager_node",
                name="stage_manager",
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
