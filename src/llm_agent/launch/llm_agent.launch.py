"""Launch LLM agent node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_agent',
            executable='llm_node',
            name='llm_agent_node',
            output='screen',
        ),
    ])
