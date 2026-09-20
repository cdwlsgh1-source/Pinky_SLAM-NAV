import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('zone_traffic_control')
    params_file = os.path.join(pkg_share, 'config', 'zone_params.yaml')

    return LaunchDescription([
        Node(
            package='zone_traffic_control',
            executable='zone_manager_node',
            name='zone_manager_node',
            output='screen',
            parameters=[params_file],
        )
    ])
