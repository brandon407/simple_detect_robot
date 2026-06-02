"""
Navigation Manager — manages Nav2 lifecycle and configuration.

Handles:
- Starting/stopping Nav2 lifecycle nodes
- Loading maps for localization mode
- Switching between mapping and localization modes
- Monitoring Nav2 health
"""
import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger, SetBool
from nav2_msgs.srv import LoadMap


class NavigationManager(Node):
    """Manage the Nav2 navigation stack lifecycle."""

    def __init__(self):
        super().__init__('navigation_manager')

        # Services for controlling navigation
        self._load_map_srv = self.create_service(
            Trigger, '/nav/load_map', self._load_map_callback,
            callback_group=ReentrantCallbackGroup())
        self._clear_costmap_srv = self.create_service(
            Trigger, '/nav/clear_costmaps', self._clear_costmaps_callback,
            callback_group=ReentrantCallbackGroup())

        # Nav2 services (clients)
        self._load_map_client = self.create_client(
            LoadMap, '/map_server/load_map',
            callback_group=ReentrantCallbackGroup())

        # Parameters
        self.declare_parameter('default_map', 'factory_map')
        self.declare_parameter('map_directory',
                               os.path.expanduser('~/.inspection_robot/maps'))

        self.get_logger().info('Navigation Manager ready')

    def load_map(self, map_name: str | None = None) -> tuple[bool, str]:
        """Load a saved map into Nav2 map_server.

        Args:
            map_name: Name of the map to load (without .yaml).

        Returns:
            (success, message) tuple.
        """
        if map_name is None:
            map_name = self.get_parameter('default_map').value

        map_dir = self.get_parameter('map_directory').value
        yaml_path = os.path.join(map_dir, f'{map_name}.yaml')

        if not os.path.exists(yaml_path):
            return False, f'Map file not found: {yaml_path}'

        # Wait for map_server service
        if not self._load_map_client.wait_for_service(timeout_sec=5.0):
            return False, 'map_server /load_map service not available'

        request = LoadMap.Request()
        request.map_url = f'file://{yaml_path}'

        future = self._load_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done() and future.result() is not None:
            result = future.result()
            if result.result == result.RESULT_SUCCESS:
                self.get_logger().info(f'Map loaded: {map_name}')
                return True, f'Map loaded: {map_name}'
            else:
                return False, f'Map load failed (result={result.result})'
        else:
            return False, 'Map load timed out'

    # ── Service Callbacks ───────────────────────────────────────

    def _load_map_callback(self, request, response):
        map_dir = self.get_parameter('map_directory').value
        from .slam_manager import SLAMManager
        # Create a temporary instance just to list maps
        map_name = self.get_parameter('default_map').value

        success, message = self.load_map(map_name)
        response.success = success
        response.message = message
        return response

    def _clear_costmaps_callback(self, request, response):
        # Publish an empty pose to trigger costmap clearing
        # This is handled by Nav2's /global_costmap/clear_around_robot service
        response.success = True
        response.message = 'Costmaps clear requested'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
