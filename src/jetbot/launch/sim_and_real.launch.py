import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jetbot')
    hw_params = os.path.join(pkg_share, 'config', 'hardware.yaml')

    port = LaunchConfiguration('port')
    hardware = LaunchConfiguration('hardware')

    # Gazebo + robot_state_publisher + ros_gz bridge + RViz (unchanged)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'display_sim.launch.py')))

    # Real robot: /cmd_vel -> Arduino Uno -> L298N
    arduino = Node(
        package='jetbot',
        executable='arduino_driver',
        name='arduino_driver',
        output='screen',
        parameters=[hw_params, {'port': port}],
        condition=IfCondition(hardware),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Arduino serial port'),
        DeclareLaunchArgument(
            'hardware', default_value='true',
            description='false = simulation only'),
        sim,
        arduino,
    ])
