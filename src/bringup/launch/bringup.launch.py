import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Launch args
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:57600',
        description='pakai /dev/ttyACM0:115200 klo pake USB.')

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url', default_value='',
        description='Forward MAVLink ke ground station (cth le udp://@192.168.1.10:14550). '
                    'Kosong = ga diforward.')

    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system', default_value='1',
        description='MAVLink system ID FCU.')

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='camera/image_raw',
        description='Topic sensor_msgs/Image yang dibaca target_recognition. '
                    'Meng-override topics.camera di target_params.yaml.')

    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Jalankan driver kamera USB (v4l2_camera). Set false klo '
                    'kameranya dijalankan terpisah / lagi tes tanpa kamera.')

    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='Device V4L2 kamera USB.')

    # bagian ini utk template sim.launch
    target_params_file_arg = DeclareLaunchArgument(
        'target_params_file',
        default_value=os.path.join(
            get_package_share_directory('target_recognition'),
            'config', 'target_params.yaml'),
        description='File parameter target_recognition. Default = profil HARDWARE '
                    '(kalibrasi kamera USB asli). sim.launch.py menggantinya dengan '
                    'target_params_sim.yaml — intrinsik Gazebo beda 2.28x.')

    drop_servo_channel_arg = DeclareLaunchArgument(
        'drop_servo_channel', default_value='[10]',
        description='Channel servo AUX untuk bay drop, sebagai list. Default [10] '
                    'sesuai wiring hardware. Di SITL channel 10 dipakai pitch gimbal, '
                    'jadi sim.launch.py memakai [12].')

    # --- 1. MAVROS (apm.launch adalah file XML) ---
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
        }.items()
    )

    # --- 2. Kamera USB ---
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        namespace='camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'time_per_frame': [1, 30],
        }],
    )

    # --- 3. Target recognition ---
    target_recognition_node = Node(
        package='target_recognition',
        executable='target_detection',
        name='target_recognition_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('target_params_file'),
            {'topics.camera': LaunchConfiguration('camera_topic')},
        ],
    )

    # --- 4. Airdrop planner ---
    airdrop_params = os.path.join(
        get_package_share_directory('airdrop_planning'), 'config', 'airdrop_params.yaml')

    airdrop_planner_node = Node(
        package='airdrop_planning',
        executable='airdrop_planner_node',
        name='airdrop_planner',
        output='screen',
        emulate_tty=True,
        parameters=[
            airdrop_params,
            # Override di ATAS file YAML: airdrop_params.yaml tetap satu-satunya
            # sumber kebenaran penerbangan, hanya channel bay yang bisa digeser
            # untuk SITL.
            {'drop.servo_channel_bay': ParameterValue(
                LaunchConfiguration('drop_servo_channel'), value_type=List[int])},
        ],
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        camera_topic_arg,
        use_camera_arg,
        video_device_arg,
        target_params_file_arg,
        drop_servo_channel_arg,
        mavros_launch,
        camera_node,
        target_recognition_node,
        airdrop_planner_node,
    ])