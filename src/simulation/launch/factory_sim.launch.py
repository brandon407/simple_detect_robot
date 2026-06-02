"""Launch Gazebo factory simulation world."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('inspection_simulation')
    world_file = os.path.join(sim_pkg, 'worlds', 'industrial_factory.world')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(sim_pkg, 'models') + ':${GAZEBO_MODEL_PATH}'),

        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo_server',
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so',
                       '-s', 'libgazebo_ros_factory.so',
                       world_file],
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_client',
            output='screen',
        ),
    ])
