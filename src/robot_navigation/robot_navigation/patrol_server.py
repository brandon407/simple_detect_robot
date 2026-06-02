"""
Patrol Mission Action Server — ROS2 Action server for patrol missions.

Accepts PatrolMission.action goals and orchestrates the WaypointPatrol node
to execute sequential waypoint navigation with feedback.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from inspection_msgs.action import PatrolMission


class PatrolServer(Node):
    """Action server for patrol mission management."""

    def __init__(self):
        super().__init__('patrol_server')

        self._action_server = ActionServer(
            self,
            PatrolMission,
            '/patrol/execute',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Reference to waypoint patrol executor (will be set after creation)
        from .waypoint_patrol import WaypointPatrol
        self._patrol = WaypointPatrol()
        self._patrol_executor = None
        self._active_goal_handle = None
        self._cancel_requested = False

        # Feedback timer
        self._fb_timer = self.create_timer(
            0.5, self._publish_periodic_feedback, callback_group=ReentrantCallbackGroup())

        self.get_logger().info('Patrol Action Server ready (/patrol/execute)')

    # ── Action Server Callbacks ─────────────────────────────────

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Validate and accept/reject incoming patrol goals."""
        waypoints = goal_request.waypoints

        if not waypoints:
            self.get_logger().error('Rejected: empty waypoint list')
            return GoalResponse.REJECT

        if len(waypoints) > 100:
            self.get_logger().error(f'Rejected: too many waypoints ({len(waypoints)})')
            return GoalResponse.REJECT

        # Check if currently busy
        state = self._patrol.get_state()
        from .waypoint_patrol import PatrolState
        if state not in (PatrolState.IDLE, PatrolState.COMPLETED, PatrolState.ABORTED):
            self.get_logger().warn(f'Rejected: patrol busy (state={state.value})')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepted patrol: {len(waypoints)} waypoints, '
            f'loop={goal_request.loop_mode}, stay={goal_request.stay_duration}s')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Handle patrol cancellation request."""
        self.get_logger().info('Cancel request received')
        self._cancel_requested = True
        self._patrol.cancel_patrol()
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """Execute patrol mission — main action logic."""
        self._active_goal_handle = goal_handle
        self._cancel_requested = False
        request = goal_handle.request

        # Start patrol with inspection integration
        inspection_modes = list(request.inspection_modes) if request.inspection_modes else []
        success = self._patrol.start_patrol(
            waypoints=list(request.waypoints),
            loop_mode=request.loop_mode,
            stay_duration=request.stay_duration,
            inspection_modes=inspection_modes,
        )

        if not success:
            goal_handle.abort()
            result = PatrolMission.Result()
            result.success = False
            result.summary = 'Failed to start patrol'
            return result

        # Feedback loop — wait for completion or cancellation
        feedback_msg = PatrolMission.Feedback()
        from .waypoint_patrol import PatrolState

        rate = self.create_rate(10)
        while rclpy.ok() and not self._cancel_requested:
            progress = self._patrol.get_progress()

            feedback_msg.current_waypoint = progress['current_waypoint']
            feedback_msg.distance_remaining = progress['distance_remaining']
            feedback_msg.current_state = progress['state']
            goal_handle.publish_feedback(feedback_msg)

            if progress['state'] in (PatrolState.COMPLETED.value, PatrolState.ABORTED.value):
                break

            rate.sleep()

        # Build result
        result = PatrolMission.Result()
        progress = self._patrol.get_progress()

        if progress['state'] == PatrolState.COMPLETED.value and not self._cancel_requested:
            result.success = True
            result.waypoints_visited = progress['total_waypoints']
            result.summary = (
                f'Patrol complete: {result.waypoints_visited}/{len(request.waypoints)} '
                f'waypoints visited successfully.'
            )
            goal_handle.succeed()
            self.get_logger().info(result.summary)
        else:
            result.success = False
            result.waypoints_visited = progress['current_waypoint']
            result.summary = (
                f'Patrol {"cancelled" if self._cancel_requested else "failed"}: '
                f'{result.waypoints_visited}/{len(request.waypoints)} waypoints visited.'
            )
            if self._cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self.get_logger().warn(result.summary)

        self._active_goal_handle = None
        return result

    def _publish_periodic_feedback(self):
        """Publish feedback even when no active goal (for monitoring)."""
        if self._active_goal_handle and self._active_goal_handle.is_active:
            # Feedback is published in the execute callback loop
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PatrolServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node._patrol)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._patrol.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
