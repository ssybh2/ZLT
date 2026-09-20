import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("soem_bringup"),
        "config",
        "config.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "interface",
            default_value="enp1s0",
            description="Dedicated EtherCAT network interface (check with ip -br link)",
        ),
        DeclareLaunchArgument(
            "rt_cpu",
            default_value="7",
            description="CPU index for the EtherCAT real-time thread",
        ),
        DeclareLaunchArgument(
            "non_rt_cpus",
            default_value="0,1,2,3,4,5,6",
            description="CPU indexes used by non-real-time threads",
        ),
        Node(
            package="soem_wrapper",
            executable="soem_backend",
            name="soem_backend",
            parameters=[{
                "interface": ParameterValue(
                    LaunchConfiguration("interface"), value_type=str
                ),
                "rt_cpu": ParameterValue(
                    LaunchConfiguration("rt_cpu"), value_type=int
                ),
                "non_rt_cpus": ParameterValue(
                    LaunchConfiguration("non_rt_cpus"), value_type=str
                ),
                "config_file": config_file,
            }],
            output="screen",
        ),
    ])
