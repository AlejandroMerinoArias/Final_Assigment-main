#!/usr/bin/env python3
"""
Mock Map Publisher for Exploration Demo.

Publishes a simple OccupancyGrid with walls, free space, and unknown regions
to visualize the exploration_manager in RViz2.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np


class MockMapPublisher(Node):
    def __init__(self):
        super().__init__('mock_map_publisher')

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        # Map parameters
        self.width = 100
        self.height = 100
        self.resolution = 0.1  # 10cm per cell
        self.origin_x = -5.0
        self.origin_y = -5.0

        # Initialize map data
        self.map_data = self._create_demo_map()
        self.exploration_step = 0

        self.get_logger().info('Mock Map Publisher started.')
        self.get_logger().info(f'  Map size: {self.width}x{self.height} @ {self.resolution}m/cell')

    def _create_demo_map(self):
        """Create a demo map with walls and unknown regions."""
        # Start with everything unknown (-1)
        data = np.full((self.height, self.width), -1, dtype=np.int8)

        # Create a "revealed" area in the center (free space)
        center_y, center_x = self.height // 2, self.width // 2
        for y in range(center_y - 15, center_y + 15):
            for x in range(center_x - 15, center_x + 15):
                if 0 <= y < self.height and 0 <= x < self.width:
                    data[y, x] = 0  # Free

        # Add some walls (occupied = 100)
        # Horizontal wall
        for x in range(center_x - 10, center_x + 10):
            if 0 <= center_y - 5 < self.height:
                data[center_y - 5, x] = 100
        # Vertical wall segment
        for y in range(center_y - 10, center_y - 5):
            if 0 <= center_x + 5 < self.width:
                data[y, center_x + 5] = 100

        return data

    def _expand_known_area(self):
        """Simulate exploration by expanding known area."""
        # Expand the boundary of known area slightly
        new_data = self.map_data.copy()
        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                if self.map_data[y, x] == 0:  # If free
                    # Check neighbors for unknown and reveal them as free
                    for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        ny, nx = y + dy, x + dx
                        if self.map_data[ny, nx] == -1:
                            # 80% chance free, 20% wall
                            if np.random.random() < 0.95:
                                new_data[ny, nx] = 0
                            else:
                                new_data[ny, nx] = 100
        self.map_data = new_data

    def publish_map(self):
        """Publish the occupancy grid."""
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(msg)

        # Simulate exploration every few publishes
        self.exploration_step += 1
        if self.exploration_step % 3 == 0:
            self._expand_known_area()
            self.get_logger().info('Map expanded (simulating exploration).')


def main(args=None):
    rclpy.init(args=args)
    node = MockMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
