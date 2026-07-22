from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('drop_mechanism'),
        'config', 'drop_params.yaml')

    return LaunchDescription([
        Node(
            package='drop_mechanism',
            executable='drop_relay_node',
            name='drop_relay',
            output='screen',
            parameters=[params],
        ),
    ])
