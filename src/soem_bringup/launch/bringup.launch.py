from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('soem_bringup'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='soem_wrapper',
            executable='soem_backend',
            name='soem_backend',
            parameters=[{
                'interface': "enx6c1ff7dfab15",
                'rt_cpu': 1,
                'non_rt_cpus': "0,2,3,4,5,6,7,8,9,10,11,12,13,14,15",
                'config_file': config_file
            }],
            output='screen'
        )
    ])
