"""
Save the SLAM Toolbox map as <name>.pgm + <name>.yaml.

Usage: ros2 run guide_dog_navigation save_map [name]
A relative name is taken from the current directory (slam_toolbox writes the
files from its own working directory, so the path is made absolute here).
Without a name: map_<timestamp> in the current directory. To use a new map
with navigation, save it into src/guide_dog_navigation/maps and rebuild, or
pass map:=<path to yaml> to the launch file.
"""

from datetime import datetime
import os
import sys

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap


class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /slam_toolbox/save_map service...')

    def save_map(self, map_path):
        request = SaveMap.Request()
        request.name.data = map_path

        self.get_logger().info(f'Saving map to: {map_path}')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Map saved successfully!')
            return True
        self.get_logger().error('Failed to save map')
        return False


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        map_name = sys.argv[1]
    else:
        map_name = 'map_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    map_name = os.path.abspath(os.path.expanduser(map_name))
    os.makedirs(os.path.dirname(map_name), exist_ok=True)

    saver = MapSaver()
    success = saver.save_map(map_name)

    saver.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
