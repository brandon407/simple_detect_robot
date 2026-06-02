"""Launch Gazebo factory simulation world."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = get_package_share_directory('inspection_simulation')
    world_file = os.path.join(sim_pkg, 'worlds', 'industrial_factory.world')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(sim_pkg, 'models') + ':${GAZEBO_MODEL_PATH}'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
            launch_arguments={'world': world_file}.items(),
        ),
    ])
