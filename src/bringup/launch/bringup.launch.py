import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Gazebo Simulation
    # Assuming the SDF is in your current working directory or Gazebo resource path
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-r', 'alti_transition_runway.sdf'],
        output='screen'
    )

    # 2. ArduPilot SITL
    # Converted "output add 127.0.0.1:14551" into the standard --out startup flag
    sitl_param_file = os.path.expanduser('~/FW_RSC_2026/src/simulation/config/alti_transition_quad.param')
    sitl_process = ExecuteProcess(
        cmd=[
            'sim_vehicle.py', '-v', 'ArduPlane', '--model', 'JSON',
            f'--add-param-file={sitl_param_file}',
            '--console', '--map', '--out=127.0.0.1:14551'
        ],
        output='screen'
    )

    # 3. MAVROS
    # Uses AnyLaunchDescriptionSource because apm.launch is typically an XML file in ROS 2
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
        ),
        launch_arguments={'fcu_url': 'udp://127.0.0.1:14551@14555'}.items()
    )

    # --- DELAY MAVROS ---
    # Waits 15 seconds before launching MAVROS so SITL can bind to the UDP ports
    delayed_mavros_launch = TimerAction(
        period=15.0,
        actions=[mavros_launch]
    )

    # 4. Camera Node (ros_gz_bridge)
    # Note: If bridge.yaml is inside a package, replace 'bridge.yaml' with an os.path.join 
    # using get_package_share_directory() for better reliability regardless of where you launch from.
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'config_file': 'bridge.yaml'}] 
    )

    # 5. Target Recognition Launch
    target_recognition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('target_recognition'), 'launch', 'target_recognition.launch.py')
        )
    )

    # 6. Airdrop Planner Launch
    airdrop_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airdrop_planning'), 'launch', 'airdrop_planning.launch.py')
        )
    )

    # --- DELAY CUSTOM NODES ---
    # Waits 20 seconds before launching your code so MAVROS is fully initialized
    delayed_custom_nodes = TimerAction(
        period=20.0,
        actions=[target_recognition_launch, airdrop_planner_node]
    )

    return LaunchDescription([
        gazebo_process,
        sitl_process,
        gz_bridge_node,
        delayed_mavros_launch,
        delayed_custom_nodes
    ])
