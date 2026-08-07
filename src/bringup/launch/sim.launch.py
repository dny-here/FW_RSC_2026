"""Simulasi SITL penuh untuk slice airdrop — satu perintah.

    bash -c 'source install/setup.bash && ros2 launch bringup sim.launch.py'

Launch ini HANYA berisi scaffolding simulasi: Gazebo, ArduPilot SITL, jembatan
kamera gz->ROS, lalu meng-include bringup.launch.py apa adanya untuk MAVROS dan
kedua node terbang. Node TIDAK didefinisikan ulang di sini — sim yang
menjalankan definisi node berbeda dari yang diterbangkan tidak membuktikan apa
pun tentang penerbangan.

Yang harus dikerjakan manual (di luar launch ini):
  1. Mission Planner -> connect UDP port 14550
  2. Load WP File -> missions/dz12_survey.waypoints -> Write
  3. Arm, lalu mode AUTO

Lihat docs/superpowers/specs/2026-08-07-sitl-simulation-bringup-design.md.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Endpoint MAVLink. Dua --out terpisah: MAVROS dan Mission Planner masing-masing
# butuh port sendiri — berbagi satu port membuat keduanya berebut socket UDP
# yang sama dan salah satu diam-diam kehilangan telemetri.
MAVROS_PORT = 14551
GCS_PORT = 14550

# Channel servo bay untuk SITL. Wiring hardware memakai 10, tapi di SITL output
# 10 tersambung ke sendi pitch gimbal lewat ArduPilotPlugin <control channel="9">
# (0-indexed), sejalan dengan SERVO10_FUNCTION 9 (Mount1 Pitch). Lihat catatan
# panjang di simulation/config/alti_transition_quad.param.
SIM_DROP_SERVO_CHANNEL = '[12]'


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _launch_setup(context, *args, **kwargs):
    sim_share = get_package_share_directory('simulation')
    bringup_share = get_package_share_directory('bringup')
    target_share = get_package_share_directory('target_recognition')

    headless = _truthy(context, 'headless')
    mavproxy_gui = _truthy(context, 'mavproxy_gui')
    wipe_params = _truthy(context, 'wipe_params')
    world = LaunchConfiguration('world').perform(context)

    # ------------------------------------------------------------ 1. Gazebo
    # World dilewatkan sebagai path ABSOLUT supaya tetap ketemu walau
    # GZ_SIM_RESOURCE_PATH salah. Hanya URI model:// yang butuh resource path.
    world_path = world if os.path.isabs(world) else os.path.join(
        sim_share, 'worlds', world)

    gz_cmd = ['gz', 'sim', '-v4', '-r']
    if headless:
        gz_cmd.append('-s')
    gz_cmd.append(world_path)

    gazebo = ExecuteProcess(cmd=gz_cmd, output='screen', name='gazebo')

    # -------------------------------------------------- 2. jembatan kamera
    # config_file path ABSOLUT dari share/. Dulu 'bridge.yaml' relatif, jadi
    # launch hanya jalan kalau kebetulan dijalankan dari root workspace.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(bringup_share, 'config', 'bridge.yaml'),
        }],
    )

    # -------------------------------------------------- 3. ArduPilot SITL
    sitl_cmd = [
        'sim_vehicle.py', '-v', 'ArduPlane', '--model', 'JSON',
        '--add-param-file=' + os.path.join(
            sim_share, 'config', 'alti_transition_quad.param'),
        '--out=127.0.0.1:{}'.format(MAVROS_PORT),
        '--out=127.0.0.1:{}'.format(GCS_PORT),
    ]
    if mavproxy_gui:
        sitl_cmd += ['--console', '--map']
    if wipe_params:
        sitl_cmd.append('-w')

    sitl = ExecuteProcess(cmd=sitl_cmd, output='screen', name='sitl')

    delayed_sitl = TimerAction(
        period=LaunchConfiguration('sitl_delay'), actions=[sitl])

    # ---------------------- 4. MAVROS + kedua node (via bringup.launch.py)
    # gcs_url dibiarkan kosong: Mission Planner bicara langsung ke MAVProxy di
    # port GCS, tidak lewat MAVROS.
    flight_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'fcu_url': 'udp://127.0.0.1:{}@14555'.format(MAVROS_PORT),
            'use_camera': 'false',          # kamera datang dari gz bridge
            'camera_topic': 'camera_node',
            'target_params_file': os.path.join(
                target_share, 'config', 'target_params_sim.yaml'),
            'drop_servo_channel': SIM_DROP_SERVO_CHANNEL,
        }.items(),
    )

    delayed_stack = TimerAction(
        period=LaunchConfiguration('stack_delay'), actions=[flight_stack])

    # ----------------------------------------------- 5. matikan bersama-sama
    # Tanpa ini, SITL yang mati di detik ke-5 meninggalkan Gazebo dan kedua node
    # tetap hidup: stack yang tampak sehat tapi tidak melakukan apa-apa.
    shutdown_on_gazebo_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo berhenti'))]))

    shutdown_on_sitl_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=sitl,
            on_exit=[EmitEvent(event=Shutdown(reason='SITL berhenti'))]))

    return [
        gazebo,
        gz_bridge,
        delayed_sitl,
        delayed_stack,
        shutdown_on_gazebo_exit,
        shutdown_on_sitl_exit,
    ]


def generate_launch_description():
    sim_share = get_package_share_directory('simulation')
    sim_prefix = get_package_prefix('simulation')

    # ------------------------------------------------------------------ args
    world_arg = DeclareLaunchArgument(
        'world', default_value='alti_transition_runway.sdf',
        description='Nama file world di simulation/worlds, atau path absolut.')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true = jalankan server Gazebo tanpa GUI (gz sim -s).')

    wipe_params_arg = DeclareLaunchArgument(
        'wipe_params', default_value='false',
        description='true = sim_vehicle.py -w, hapus parameter EEPROM lebih '
                    'dulu. Pakai SEKALI setelah alti_transition_quad.param '
                    'berubah — kalau tidak, nilai lama di eeprom.bin tetap '
                    'menang dan blok MNT1/SERVO12 yang baru tidak berlaku.')

    mavproxy_gui_arg = DeclareLaunchArgument(
        'mavproxy_gui', default_value='true',
        description='true = sim_vehicle.py --console --map.')

    sitl_delay_arg = DeclareLaunchArgument(
        'sitl_delay', default_value='5.0',
        description='Detik menunggu ArduPilotPlugin Gazebo mendengarkan sebelum '
                    'SITL menyambung. Penundaan ini MENANGGUNG BEBAN — SITL yang '
                    'start duluan gagal menyambung ke model JSON.')

    stack_delay_arg = DeclareLaunchArgument(
        'stack_delay', default_value='12.0',
        description='Detik sebelum MAVROS + kedua node dinyalakan. TIDAK '
                    'menanggung beban: MAVROS mencoba ulang selamanya dan '
                    'planner menurun dengan anggun saat service belum siap. Ada '
                    'semata untuk meredam derau log; aman di-nol-kan.')

    # ------------------------------------------------------------------- env
    # Di-set eksplisit, TIDAK mengandalkan hook paket simulation:
    # src/simulation/CMakeLists.txt menyembunyikan ament_environment_hooks() dan
    # ament_package() di balik if(${ament_cmake_FOUND}) — kondisional yang sudah
    # pernah diam-diam jadi NOTFOUND di repo ini. Men-set di sini membuat cache
    # rusak berbiaya satu rebuild, bukan satu sesi debugging.
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join([p for p in [
            os.path.join(sim_share, 'models'),
            os.path.join(sim_share, 'worlds'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ] if p]))

    gz_plugin_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.pathsep.join([p for p in [
            os.path.join(sim_prefix, 'lib', 'simulation'),
            os.path.join(sim_prefix, 'lib'),
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ] if p]))

    return LaunchDescription([
        world_arg,
        headless_arg,
        wipe_params_arg,
        mavproxy_gui_arg,
        sitl_delay_arg,
        stack_delay_arg,
        gz_resource_env,
        gz_plugin_env,
        OpaqueFunction(function=_launch_setup),
    ])
