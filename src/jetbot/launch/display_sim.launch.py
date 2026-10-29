import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'jetbot'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file_path = os.path.join(pkg_share, 'urdf', 'jetbot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'jetbot.rviz')

    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo Sim (empty world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # 3. Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content, '-name', 'jetbot', '-z', '0.05']
    )

    # 4. Gazebo <-> ROS 2 bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(pkg_share, 'config', 'bridge_config.yaml')}]
    )

    # 5. RViz2 with the JetBot layout (robot, real LiDAR, safety zone)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        bridge,
        rviz_node
    ])
