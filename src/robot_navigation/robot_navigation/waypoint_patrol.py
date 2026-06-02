"""
Waypoint Patrol Node — executes sequential patrol missions via Nav2.

Uses Nav2 NavigateToPose action to drive the robot through an ordered
list of waypoints with configurable stay duration and loop support.
"""
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


class PatrolState(Enum):
    IDLE = 'idle'
    NAVIGATING = 'navigating'
    WAITING = 'waiting'
    COMPLETED = 'completed'
    ABORTED = 'aborted'


class WaypointPatrol(Node):
    """Execute waypoint-based patrol missions using Nav2."""

    def __init__(self):
        super().__init__('waypoint_patrol')

        # Nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=ReentrantCallbackGroup())

        # TF for current pose tracking
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Odom subscriber for distance estimation
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        # State
        self._state = PatrolState.IDLE
        self._waypoints: list[Pose] = []
        self._current_idx = 0
        self._loop_mode = False
        self._stay_duration = 5.0
        self._arrival_tolerance = 0.3  # meters
        self._current_goal_handle = None
        self._last_odom: Odometry | None = None
        self._nav_goal_future = None

        # Feedback timer
        self._fb_timer = self.create_timer(1.0, self._publish_feedback, callback_group=ReentrantCallbackGroup())

        # Status publishers
        self._state_pub = self.create_publisher(Odometry, '/patrol/state', 10)  # type hint

        # Wait for Nav2 action server
        self.get_logger().info('WaypointPatrol node started, waiting for Nav2...')
        self._wait_for_nav2()

    def _wait_for_nav2(self):
        """Wait for Nav2 action server to become available."""
        if self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Nav2 action server connected')
        else:
            self.get_logger().warn('Nav2 action server not available, retrying...')
            self.create_timer(2.0, lambda: self._wait_for_nav2_retry(), callback_group=ReentrantCallbackGroup())

    def _wait_for_nav2_retry(self):
        if self._nav_client.server_is_ready():
            self.get_logger().info('Nav2 action server connected')
        else:
            self.get_logger().warn('Still waiting for Nav2...')

    def _odom_callback(self, msg: Odometry):
        self._last_odom = msg

    # ── Public API ──────────────────────────────────────────────

    def start_patrol(self, waypoints: list[Pose], loop_mode: bool = False,
                     stay_duration: float = 5.0) -> bool:
        """Start a new patrol mission.

        Args:
            waypoints: Ordered list of target poses.
            loop_mode: If True, continuously loop through waypoints.
            stay_duration: Seconds to pause at each waypoint.

        Returns:
            True if patrol started successfully.
        """
        if self._state not in (PatrolState.IDLE, PatrolState.COMPLETED, PatrolState.ABORTED):
            self.get_logger().warn(f'Cannot start patrol: current state={self._state.value}')
            return False

        if not waypoints:
            self.get_logger().error('No waypoints provided')
            return False

        self._waypoints = list(waypoints)
        self._loop_mode = loop_mode
        self._stay_duration = stay_duration
        self._current_idx = 0
        self._state = PatrolState.IDLE

        self.get_logger().info(
            f'Starting patrol: {len(waypoints)} waypoints, '
            f'loop={loop_mode}, stay={stay_duration}s')

        self._navigate_to_next()
        return True

    def cancel_patrol(self) -> bool:
        """Cancel the current patrol mission."""
        if self._state not in (PatrolState.NAVIGATING, PatrolState.WAITING):
            return False
        self._cancel_current_goal()
        self._state = PatrolState.ABORTED
        self.get_logger().info('Patrol cancelled')
        return True

    def get_state(self) -> PatrolState:
        return self._state

    def get_progress(self) -> dict:
        return {
            'state': self._state.value,
            'current_waypoint': self._current_idx,
            'total_waypoints': len(self._waypoints),
            'loop_mode': self._loop_mode,
            'distance_remaining': self._compute_distance_remaining(),
        }

    # ── Navigation Logic ────────────────────────────────────────

    def _navigate_to_next(self):
        """Send navigation goal for the next waypoint."""
        if not self._waypoints:
            return

        # Check if we've completed all waypoints
        if self._current_idx >= len(self._waypoints):
            if self._loop_mode:
                self._current_idx = 0
                self.get_logger().info('Looping back to first waypoint')
            else:
                self._state = PatrolState.COMPLETED
                self.get_logger().info(f'Patrol complete: {len(self._waypoints)} waypoints visited')
                return

        wp = self._waypoints[self._current_idx]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = wp
        goal_msg.behavior_tree = ''  # Use default BT

        self.get_logger().info(
            f'Navigating to waypoint {self._current_idx + 1}/{len(self._waypoints)}: '
            f'x={wp.position.x:.2f}, y={wp.position.y:.2f}')

        self._state = PatrolState.NAVIGATING
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Waypoint {self._current_idx + 1} rejected by Nav2')
            self._state = PatrolState.ABORTED
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info(f'Waypoint {self._current_idx + 1} accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Called when Nav2 finishes navigating to a waypoint."""
        result = future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Arrived at waypoint {self._current_idx + 1}')
            self._state = PatrolState.WAITING
            self._current_goal_handle = None

            # Stay at waypoint for configured duration
            if self._stay_duration > 0:
                self.get_logger().info(f'Waiting {self._stay_duration:.1f}s at waypoint...')
                self.create_timer(
                    self._stay_duration,
                    self._on_stay_complete,
                    callback_group=ReentrantCallbackGroup())
            else:
                self._on_stay_complete()
        else:
            self.get_logger().error(
                f'Navigation to waypoint {self._current_idx + 1} failed (status={status})')
            self._state = PatrolState.ABORTED
            self._current_goal_handle = None

    def _on_stay_complete(self):
        """Proceed to next waypoint after stay duration."""
        self._current_idx += 1
        self._navigate_to_next()

    def _cancel_current_goal(self):
        """Cancel the active navigation goal."""
        if self._current_goal_handle:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Goal cancelled'))
            self._current_goal_handle = None

    def _compute_distance_remaining(self) -> float:
        """Estimate remaining distance to current waypoint."""
        if self._last_odom is None or self._state != PatrolState.NAVIGATING:
            return 0.0
        if self._current_idx >= len(self._waypoints):
            return 0.0

        wp = self._waypoints[self._current_idx]
        dx = wp.position.x - self._last_odom.pose.pose.position.x
        dy = wp.position.y - self._last_odom.pose.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def _publish_feedback(self):
        """Periodic status feedback."""
        progress = self.get_progress()
        # Would publish to a dedicated feedback topic in production
        self.get_logger().debug(
            f'Patrol state={progress["state"]}, '
            f'waypoint={progress["current_waypoint"]}/{progress["total_waypoints"]}, '
            f'dist_remaining={progress["distance_remaining"]:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
