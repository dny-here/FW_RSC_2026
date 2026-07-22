import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('mission_bridge'),
        'config', 'mission_bridge_params.yaml')
    return LaunchDescription([
        Node(
            package='mission_bridge',
            executable='mission_bridge_node',
            name='mission_bridge',
            output='screen',
            parameters=[params],
        )
    ])
