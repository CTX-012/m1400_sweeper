import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

#  gamepad / Gazebo panel -> /cmd_vel_raw -> obstacle_guard -> /cmd_vel -> Gazebo robot
#                                                                       -> arduino_driver -> real robot
#  real RPLIDAR -> /real/scan   -> obstacle_guard   (obstacles ~6 in and taller)
#  USB camera   -> floor_camera -> /camera/scan -> obstacle_guard   (low obstacles)


def generate_launch_description():
    pkg_share = get_package_share_directory('jetbot')
    hw_params = os.path.join(pkg_share, 'config', 'hardware.yaml')
    joy_params = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    guard_params = os.path.join(pkg_share, 'config', 'guard.yaml')

    port = LaunchConfiguration('port')
    hardware = LaunchConfiguration('hardware')
    joystick = LaunchConfiguration('joystick')
    lidar = LaunchConfiguration('lidar')
    lidar_port = LaunchConfiguration('lidar_port')
    camera = LaunchConfiguration('camera')
    camera_device = LaunchConfiguration('camera_device')

    # Gamepad commands arrive continuously, so stop if they ever stop.
    # Gazebo-panel commands only arrive on key presses, so hold the last one.
    cmd_timeout = PythonExpression(
        ["0.5 if '", joystick, "'.lower() == 'true' else 0.0"])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'display_sim.launch.py')))

    rplidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
        remappings=[('scan', '/real/scan')],
        condition=IfCondition(lidar),
    )

    floor_camera = Node(
        package='jetbot',
        executable='floor_camera',
        name='floor_camera',
        output='screen',
        parameters=[{
            'device': camera_device,
            'calib_file': '~/.ros/jetbot_camera_floor.yaml',
        }],
        condition=IfCondition(camera),
    )

    guard = Node(
        package='jetbot',
        executable='obstacle_guard',
        name='obstacle_guard',
        output='screen',
        parameters=[guard_params, {
            'use_sim_time': True,
            'cmd_timeout': cmd_timeout,
            'require_scan': lidar,
            'use_camera': camera,
        }],
    )

    arduino = Node(
        package='jetbot',
        executable='arduino_driver',
        name='arduino_driver',
        output='screen',
        parameters=[hw_params, {'port': port, 'cmd_timeout': cmd_timeout}],
        condition=IfCondition(hardware),
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_params],
        condition=IfCondition(joystick),
    )
    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[joy_params],
        remappings=[('cmd_vel', '/cmd_vel_raw')],
        condition=IfCondition(joystick),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Arduino serial port'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            description='RPLIDAR serial port'),
        DeclareLaunchArgument('hardware', default_value='true', description='false = no Arduino'),
        DeclareLaunchArgument('lidar', default_value='true', description='false = no real LiDAR (guard off)'),
        DeclareLaunchArgument('camera', default_value='true', description='false = no low-obstacle camera'),
        DeclareLaunchArgument('camera_device', default_value='0', description='camera /dev/videoN'),
        DeclareLaunchArgument(
            'joystick', default_value='true',
            description='true = gamepad, false = Gazebo Teleop panel (topic /cmd_vel_raw)'),
        sim,
        rplidar,
        floor_camera,
        guard,
        arduino,
        joy,
        teleop,
    ])
