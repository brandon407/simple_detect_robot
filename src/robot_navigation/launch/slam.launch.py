"""Launch SLAM Toolbox for online mapping."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('robot_bringup')
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),
    ])
