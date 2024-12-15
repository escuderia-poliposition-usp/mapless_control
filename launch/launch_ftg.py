from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mapless_control'), 'config', 'config.yaml')
    return LaunchDescription([
        Node(
            package='mapless_control',
            executable='follow_the_gap',
            name='follow_the_gap',
            parameters=[config_path]
        )
    ])
