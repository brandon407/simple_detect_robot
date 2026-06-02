"""Meter reader - instrument gauge reading recognition."""
import rclpy
from rclpy.node import Node


class MeterReaderNode(Node):
    """ROS2 node for instrument gauge reading."""

    def __init__(self):
        super().__init__('meter_reader')
        self.get_logger().info('Meter reader node initialized (stub)')
        # Full implementation in Phase 3


def main(args=None):
    rclpy.init(args=args)
    node = MeterReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
