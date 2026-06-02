"""Waypoint patrol node - sends navigation goals in sequence."""
import rclpy
from rclpy.node import Node


class WaypointPatrol(Node):
    """Manage waypoint-based patrol using Nav2."""

    def __init__(self):
        super().__init__('waypoint_patrol')
        self.get_logger().info('Waypoint patrol node initialized (stub)')
        # Full implementation in Phase 2


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
