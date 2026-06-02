"""
Full Inspection Demo Launch — complete system in one command.

Launches: Gazebo + Robot + Nav2 + SLAM + Patrol + All Detectors + LLM Agent

Usage:
    ros2 launch robot_bringup full_inspection_demo.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = get_package_share_directory('inspection_simulation')
    desc_pkg = get_package_share_directory('robot_description')
    bringup_pkg = get_package_share_directory('robot_bringup')

    world_file = os.path.join(sim_pkg, 'worlds', 'industrial_factory.world')
    nav2_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    insp_params = os.path.join(
        get_package_share_directory('robot_inspection'), 'config', 'inspection_params.yaml')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'inspection_robot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),

        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(sim_pkg, 'models') + ':${GAZEBO_MODEL_PATH}'),

        # ===== Layer 1: Simulation =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
            launch_arguments={'world': world_file}.items(),
        ),

        # ===== Layer 2: Robot (delayed for Gazebo init) =====
        TimerAction(period=5.0, actions=[
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': ['xacro ', xacro_file], 'use_sim_time': use_sim_time}]),
            Node(package='joint_state_publisher', executable='joint_state_publisher',
                 name='joint_state_publisher', parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_robot',
                 output='screen',
                 arguments=['-entity', 'inspection_robot', '-topic', 'robot_description',
                            '-x', '0.0', '-y', '0.0', '-z', '0.15', '-Y', '0.0']),
        ]),

        # ===== Layer 3: Nav2 + SLAM (delayed after spawn) =====
        TimerAction(period=8.0, actions=[
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 output='screen', parameters=[nav2_params]),
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 output='screen', parameters=[nav2_params]),
            Node(package='nav2_controller', executable='controller_server', name='controller_server',
                 output='screen', parameters=[nav2_params]),
            Node(package='nav2_planner', executable='planner_server', name='planner_server',
                 output='screen', parameters=[nav2_params]),
            Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
                 output='screen', parameters=[nav2_params]),
            Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
                 output='screen', parameters=[nav2_params]),
            Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                 name='waypoint_follower', output='screen', parameters=[nav2_params]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'node_names': ['map_server', 'amcl', 'controller_server',
                                             'planner_server', 'behavior_server',
                                             'bt_navigator', 'waypoint_follower']}]),
            Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                 name='slam_toolbox', output='screen', parameters=[slam_params]),
        ]),

        # ===== Layer 4: Patrol =====
        TimerAction(period=12.0, actions=[
            Node(package='robot_navigation', executable='patrol_server', name='patrol_server',
                 output='screen'),
            Node(package='robot_navigation', executable='slam_manager', name='slam_manager',
                 output='screen'),
        ]),

        # ===== Layer 5: Inspection Detectors =====
        TimerAction(period=12.0, actions=[
            Node(package='robot_inspection', executable='inspection_orchestrator',
                 name='inspection_orchestrator', output='screen',
                 parameters=[insp_params]),
            Node(package='robot_inspection', executable='defect_detector',
                 name='defect_detector', output='screen',
                 parameters=[{'processing_rate': 2.0, 'enable_mock': True}]),
            Node(package='robot_inspection', executable='meter_reader',
                 name='meter_reader', output='screen',
                 parameters=[{'processing_rate': 1.0, 'enable_mock': True}]),
            Node(package='robot_inspection', executable='safety_checker',
                 name='safety_checker', output='screen',
                 parameters=[{'processing_rate': 2.0, 'enable_mock': True}]),
        ]),

        # ===== Layer 6: LLM Agent =====
        TimerAction(period=14.0, actions=[
            Node(package='llm_agent', executable='llm_node', name='llm_agent_node',
                 output='screen'),
        ]),
    ])
