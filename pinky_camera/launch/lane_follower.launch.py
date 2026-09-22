import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pinky_camera')
    params_file = os.path.join(pkg_share, 'config', 'lane_follower_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='pinky1',
            description='로봇 네임스페이스 (pinky1 | pinky2)'),
        Node(
            package='pinky_camera',
            executable='lane_follower_node',
            name='lane_follower_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[params_file],
        ),
    ])