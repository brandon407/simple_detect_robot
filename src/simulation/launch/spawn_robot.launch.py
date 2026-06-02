"""Spawn the inspection robot in an already-running Gazebo simulation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_desc_pkg = get_package_share_directory('robot_description')
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'inspection_robot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw angle'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', xacro_file])}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-entity', 'inspection_robot',
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-Y', LaunchConfiguration('yaw'),
                '-z', '0.15',
            ],
        ),
    ])
