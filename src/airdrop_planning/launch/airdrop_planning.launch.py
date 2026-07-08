from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('airdrop_planning'),
        'config', 'airdrop_params.yaml')

    return LaunchDescription([
        Node(
            package='airdrop_planning',
            executable='airdrop_planner_node',
            name='airdrop_planner',
            output='screen',
            parameters=[params],
        ),
    ])
