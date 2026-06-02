"""Launch the full simulation: Gazebo world + robot spawn + state publisher."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    simulation_pkg = get_package_share_directory('inspection_simulation')
    robot_desc_pkg = get_package_share_directory('robot_description')
    robot_bringup_pkg = get_package_share_directory('robot_bringup')

    # Paths
    world_file = os.path.join(simulation_pkg, 'worlds', 'industrial_factory.world')
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'inspection_robot.urdf.xacro')

    return LaunchDescription([
        # Set Gazebo model path for custom models
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(simulation_pkg, 'models') + ':${GAZEBO_MODEL_PATH}'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ['xacro ', xacro_file],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]),

        # Gazebo server
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-entity', 'inspection_robot',
                '-topic', 'robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.1',
                '-Y', '0.0',
            ]),

        # Use ExecuteProcess to start Gazebo with the world
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
