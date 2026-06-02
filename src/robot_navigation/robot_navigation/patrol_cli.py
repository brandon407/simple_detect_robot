#!/usr/bin/env python3
"""
Patrol CLI — command-line tool to send patrol missions.

Usage:
    ros2 run robot_navigation patrol_cli --ros-args -p waypoints:="[[0,0,0],[1,0,0],[1,1,0]]"
    ros2 run robot_navigation patrol_cli --ros-args -p "waypoints:=[[2.0, -2.5, 0.0],[4.0, 2.0, 0.0],[-4.0, 0.0, 1.57]]"
"""
import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from inspection_msgs.action import PatrolMission


def parse_waypoints(raw: str) -> list[Pose]:
    """Parse waypoints from a JSON-like string.

    Format: [[x,y,yaw],[x,y,yaw],...]
    Yaw is in radians, optional (defaults to 0).
    """
    data = json.loads(raw)
    waypoints = []
    for wp in data:
        pose = Pose()
        pose.position.x = float(wp[0])
        pose.position.y = float(wp[1])
        pose.position.z = 0.0
        # Yaw to quaternion
        yaw = float(wp[2]) if len(wp) > 2 else 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        waypoints.append(pose)
    return waypoints


class PatrolCLI(Node):
    """Send a patrol mission via action client and print progress."""

    def __init__(self, waypoints: list[Pose], loop: bool, stay: float):
        super().__init__('patrol_cli')
        self._client = ActionClient(self, PatrolMission, '/patrol/execute')
        self._done = False

        self.get_logger().info(f'Waiting for patrol server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Patrol server not available!')
            sys.exit(1)

        goal = PatrolMission.Goal()
        goal.waypoints = waypoints
        goal.loop_mode = loop
        goal.stay_duration = stay

        self.get_logger().info(
            f'Sending patrol: {len(waypoints)} waypoints, '
            f'loop={loop}, stay={stay}s')

        self._send_goal_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self._done = True
            return

        self.get_logger().info('Goal accepted, executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[Progress] state={fb.current_state}, '
            f'waypoint={fb.current_waypoint + 1}, '
            f'dist_remaining={fb.distance_remaining:.2f}m')

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'\n{"="*50}\n'
            f'PATROL RESULT\n'
            f'  Success: {result.success}\n'
            f'  Waypoints visited: {result.waypoints_visited}\n'
            f'  Summary: {result.summary}\n'
            f'{"="*50}')
        self._done = True

    def is_done(self) -> bool:
        return self._done


def main(args=None):
    rclpy.init(args=args)

    # Parse from ROS args
    temp_node = rclpy.create_node('_patrol_cli_parser')
    temp_node.declare_parameter('waypoints', '[[2.0,2.0,0],[4.0,-2.0,0],[-3.0,-1.0,0]]')
    temp_node.declare_parameter('loop', False)
    temp_node.declare_parameter('stay', 5.0)

    raw = temp_node.get_parameter('waypoints').value
    loop = temp_node.get_parameter('loop').value
    stay = temp_node.get_parameter('stay').value
    temp_node.destroy_node()

    waypoints = parse_waypoints(raw)

    cli = PatrolCLI(waypoints, loop, stay)

    while rclpy.ok() and not cli.is_done():
        rclpy.spin_once(cli, timeout_sec=0.1)

    cli.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
