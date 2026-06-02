"""Safety checker - helmet detection, zone intrusion, fire/smoke detection."""
import rclpy
from rclpy.node import Node


class SafetyCheckerNode(Node):
    """ROS2 node for safety compliance checking."""

    def __init__(self):
        super().__init__('safety_checker')
        self.get_logger().info('Safety checker node initialized (stub)')
        # Full implementation in Phase 3


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
