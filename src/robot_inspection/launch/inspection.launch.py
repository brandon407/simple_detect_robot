"""Launch inspection orchestrator and detectors."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_inspection',
            executable='inspection_orchestrator',
            name='inspection_orchestrator',
            output='screen',
        ),
    ])
