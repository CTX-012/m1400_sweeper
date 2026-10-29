import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jetbot')
    hw_params = os.path.join(pkg_share, 'config', 'hardware.yaml')
    joy_params = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')

    port = LaunchConfiguration('port')
    hardware = LaunchConfiguration('hardware')
    joystick = LaunchConfiguration('joystick')

    # With the gamepad, commands arrive continuously, so the real robot can safely
    # stop if they ever stop (gamepad unplugged / out of range). With the Gazebo
    # GUI, commands only arrive on key presses, so the last one is held instead.
    cmd_timeout = PythonExpression(
        ["0.5 if '", joystick, "'.lower() == 'true' else 0.0"])

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
        parameters=[hw_params, {'port': port, 'cmd_timeout': cmd_timeout}],
        condition=IfCondition(hardware),
    )

    # Gamepad: /joy -> /cmd_vel (drives the sim and the real robot together)
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
        condition=IfCondition(joystick),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            description='Arduino serial port'),
        DeclareLaunchArgument(
            'hardware', default_value='true',
            description='false = simulation only'),
        DeclareLaunchArgument(
            'joystick', default_value='true',
            description='true = drive with the gamepad, false = Gazebo GUI teleop'),
        sim,
        arduino,
        joy,
        teleop,
    ])
