"""Launch patrol server and waypoint follower."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_navigation',
            executable='patrol_server',
            name='patrol_server',
            output='screen',
        ),
        Node(
            package='robot_navigation',
            executable='waypoint_patrol',
            name='waypoint_patrol',
            output='screen',
        ),
    ])
