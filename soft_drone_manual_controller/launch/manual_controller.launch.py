from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("soft_drone_manual_controller"), "config", "manual_controller.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Controller YAML file",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="When true, calculate and publish diagnostics but do not publish DSHOT commands",
            ),
            Node(
                package="soft_drone_manual_controller",
                executable="manual_controller",
                name="soft_drone_manual_controller",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
