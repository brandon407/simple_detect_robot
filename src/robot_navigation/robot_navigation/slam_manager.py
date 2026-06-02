"""
SLAM Manager — manages SLAM Toolbox lifecycle, map save/load operations.

Provides services to:
- Start/stop SLAM mapping
- Save the current map via Nav2 map_server
- Load a saved map for localization mode
"""
import os
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger


class SLAMManager(Node):
    """Manage SLAM operations: mapping, map save, map load."""

    def __init__(self):
        super().__init__('slam_manager')

        # Services
        self._save_map_srv = self.create_service(
            Trigger, '/slam/save_map', self._save_map_callback,
            callback_group=ReentrantCallbackGroup())
        self._map_status_srv = self.create_service(
            Trigger, '/slam/status', self._status_callback,
            callback_group=ReentrantCallbackGroup())

        # Map storage
        self._map_dir = os.path.expanduser('~/.inspection_robot/maps')
        os.makedirs(self._map_dir, exist_ok=True)

        # Declare parameters
        self.declare_parameter('map_name', 'factory_map')
        self.declare_parameter('map_directory', self._map_dir)

        self.get_logger().info(f'SLAM Manager ready (maps: {self._map_dir})')

    def save_map(self, map_name: str | None = None) -> tuple[bool, str]:
        """Save the current SLAM map using nav2_map_server.

        Args:
            map_name: Base name for the map files. Uses param if None.

        Returns:
            (success, message) tuple.
        """
        if map_name is None:
            map_name = self.get_parameter('map_name').value

        map_path = os.path.join(self._map_dir, map_name)

        self.get_logger().info(f'Saving map to {map_path}')

        try:
            # Use nav2_map_server's map_saver_cli
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', map_path],
                capture_output=True, text=True, timeout=30.0)

            if result.returncode == 0:
                pgm_file = f'{map_path}.pgm'
                yaml_file = f'{map_path}.yaml'
                if os.path.exists(yaml_file):
                    size_kb = os.path.getsize(pgm_file) / 1024 if os.path.exists(pgm_file) else 0
                    msg = f'Map saved: {map_name} ({size_kb:.0f} KB)'
                    self.get_logger().info(msg)
                    return True, msg
                else:
                    return False, f'Map files not created for {map_name}'
            else:
                error = result.stderr.strip() or 'unknown error'
                self.get_logger().error(f'Map save failed: {error}')
                return False, f'Map save failed: {error}'

        except subprocess.TimeoutExpired:
            return False, 'Map save timed out (30s)'
        except Exception as e:
            return False, f'Map save error: {str(e)}'

    def list_maps(self) -> list[str]:
        """List all saved maps in the map directory."""
        maps = []
        if os.path.isdir(self._map_dir):
            for f in os.listdir(self._map_dir):
                if f.endswith('.yaml'):
                    maps.append(f[:-5])  # Strip .yaml
        return sorted(maps)

    def get_map_path(self, map_name: str | None = None) -> str | None:
        """Get the full YAML path for a saved map."""
        if map_name is None:
            map_name = self.get_parameter('map_name').value
        yaml_path = os.path.join(self._map_dir, f'{map_name}.yaml')
        if os.path.exists(yaml_path):
            return yaml_path
        return None

    # ── Service Callbacks ───────────────────────────────────────

    def _save_map_callback(self, request, response):
        success, message = self.save_map()
        response.success = success
        response.message = message
        return response

    def _status_callback(self, request, response):
        maps = self.list_maps()
        response.success = True
        response.message = (
            f'SLAM Manager active. '
            f'Maps directory: {self._map_dir}. '
            f'Available maps: {", ".join(maps) if maps else "none"}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SLAMManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
