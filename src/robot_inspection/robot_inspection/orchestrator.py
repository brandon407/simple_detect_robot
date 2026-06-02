"""Inspection orchestrator - coordinates detection tasks during patrol."""
import rclpy
from rclpy.node import Node


class InspectionOrchestrator(Node):
    """Main inspection coordination node."""

    def __init__(self):
        super().__init__('inspection_orchestrator')
        self.get_logger().info('Inspection orchestrator initialized (stub)')
        # Full implementation in Phase 3


def main(args=None):
    rclpy.init(args=args)
    node = InspectionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
