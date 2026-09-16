from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("sn2031674_manual_rc"),
        "config",
        "manual_rc.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="If true, process RC but do not publish DShot commands",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="throttle_only",
            description="throttle_only or x_open_loop",
        ),
        Node(
            package="sn2031674_manual_rc",
            executable="manual_rc_dshot",
            name="sn2031674_manual_rc",
            output="screen",
            parameters=[
                config_file,
                {
                    "dry_run": LaunchConfiguration("dry_run"),
                    "mode": LaunchConfiguration("mode"),
                },
            ],
        ),
    ])
