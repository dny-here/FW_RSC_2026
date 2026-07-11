import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Locate the package directory in the ROS 2 install space
    pkg_dir = get_package_share_directory('target_recognition')

    # Build the absolute path to your YAML parameter file
    config_file = os.path.join(
        pkg_dir,
        'config',
        'target_params.yaml'
    )

    # 3. Define the Node execution parameters
    target_detection_node = Node(
        package='target_recognition',
        executable='target_detection',      # Must match the name in CMakeLists.txt (add_executable)
        name='target_recognition_node',     # Must match the name in your C++ constructor
        output='screen',                    # Prints RCLCPP_INFO logs directly to your terminal
        emulate_tty=True,                   # Ensures log colors work correctly in the terminal
        parameters=[config_file]            # Injects the YAML file into the node
    )

    # 4. Return the LaunchDescription
    return LaunchDescription([
        target_detection_node
    ])