from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("ugv_stage_sign_detector"), "config", "detector.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            DeclareLaunchArgument("templates_directory", default_value=""),
            DeclareLaunchArgument("font_path", default_value=""),
            DeclareLaunchArgument("classifier_mode", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("enable_action_dispatcher", default_value="true"),
            Node(
                package="ugv_stage_sign_detector",
                executable="stage_sign_detector_node",
                name="stage_sign_detector",
                output="screen",
                parameters=[
                    {
                        "config_path": LaunchConfiguration("config_file"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "templates_directory": LaunchConfiguration("templates_directory"),
                        "font_path": LaunchConfiguration("font_path"),
                        "classifier_mode": LaunchConfiguration("classifier_mode"),
                        "model_path": LaunchConfiguration("model_path"),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration("publish_debug_image"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="ugv_stage_sign_detector",
                executable="stage_manager_node",
                name="stage_manager",
                output="screen",
                parameters=[
                    {
                        "stage_sequence": list(range(1, 11)),
                        "stage_behaviors": [
                            "fording_and_shower",
                            "aggregate_road",
                            "side_slope",
                            "vertical_obstacle",
                            "traffic_cones",
                            "sliding_doors",
                            "bumped_road",
                            "incline_stop",
                            "fire_zone",
                            "decline_stop",
                        ],
                    }
                ],
            ),
            Node(
                package="ugv_stage_sign_detector",
                executable="stage_action_dispatcher_node",
                name="stage_action_dispatcher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_action_dispatcher")),
            ),
        ]
    )
