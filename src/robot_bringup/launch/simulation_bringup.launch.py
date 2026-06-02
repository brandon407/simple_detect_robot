"""
Full simulation bringup: Gazebo world + robot spawn + Nav2 + SLAM + Patrol.

Launches everything needed for a complete inspection patrol simulation.
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
    # Package directories
    sim_pkg = get_package_share_directory('inspection_simulation')
    desc_pkg = get_package_share_directory('robot_description')
    bringup_pkg = get_package_share_directory('robot_bringup')

    # Paths
    world_file = os.path.join(sim_pkg, 'worlds', 'industrial_factory.world')
    nav2_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'inspection_robot.urdf.xacro')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map_file', default='')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': world_file, 'extra_gazebo_args': '--verbose'}.items(),
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ['xacro ', xacro_file],
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Spawn robot (delayed to let Gazebo initialize)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-entity', 'inspection_robot',
                    '-topic', 'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.15',
                    '-Y', '0.0',
                ],
            ),
        ],
    )

    # ===== Nav2 Lifecycle Nodes =====
    nav2_nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[nav2_params, {'yaml_filename': map_file}]),
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
    ]

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen', parameters=[slam_params],
    )

    # Patrol stack
    patrol_nodes = [
        Node(package='robot_navigation', executable='patrol_server', name='patrol_server',
             output='screen'),
        Node(package='robot_navigation', executable='slam_manager', name='slam_manager',
             output='screen'),
        Node(package='robot_navigation', executable='navigation_manager',
             name='navigation_manager', output='screen'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('map_file', default_value='',
                              description='Pre-built map file for localization mode'),

        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(sim_pkg, 'models') + ':${GAZEBO_MODEL_PATH}'),

        # Layer 1: Gazebo
        gazebo_launch,

        # Layer 2: Robot model + spawn
        robot_state_pub,
        joint_state_pub,
        spawn_robot,

        # Layer 3: Nav2 + SLAM
        TimerAction(period=10.0, actions=[
            *nav2_nodes,
            slam_node,
        ]),

        # Layer 4: Patrol
        TimerAction(period=14.0, actions=patrol_nodes),
    ])
