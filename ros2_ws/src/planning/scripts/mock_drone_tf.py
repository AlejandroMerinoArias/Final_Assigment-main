#!/usr/bin/env python3
"""
Mock Drone TF Broadcaster for Exploration Demo.

Broadcasts a TF transform for `base_link` in the `map` frame.
Listens to `/exploration/strategic_goal` and moves toward it.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import math


class MockDroneTF(Node):
    def __init__(self):
        super().__init__('mock_drone_tf')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Drone state
        self.x = 0.0
        self.y = 0.0
        self.z = 1.5  # Hover altitude
        self.yaw = 0.0

        # Target (from exploration manager)
        self.target_x = 0.0
        self.target_y = 0.0
        self.has_target = False

        # Movement speed
        self.speed = 0.5  # m/s

        # Subscriptions
        self.goal_sub = self.create_subscription(
            PointStamped,
            '/exploration/strategic_goal',
            self.goal_callback,
            10
        )

        # Drone marker publisher (for visualization)
        self.marker_pub = self.create_publisher(Marker, '/drone_marker', 10)

        # Timer for TF and movement updates
        self.timer = self.create_timer(0.05, self.update)  # 20Hz

        self.get_logger().info('Mock Drone TF Broadcaster started.')
        self.get_logger().info(f'  Initial position: ({self.x}, {self.y}, {self.z})')

    def goal_callback(self, msg: PointStamped):
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self.has_target = True
        self.get_logger().info(f'New target: ({self.target_x:.2f}, {self.target_y:.2f})')

    def update(self):
        # Move toward target
        if self.has_target:
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > 0.1:
                # Move toward target
                step = min(self.speed * 0.05, dist)  # 0.05s = timer period
                self.x += (dx / dist) * step
                self.y += (dy / dist) * step
                self.yaw = math.atan2(dy, dx)
            else:
                # Reached target
                self.has_target = False
                self.get_logger().info('Reached target goal.')

        # Broadcast TF
        self._broadcast_tf()

        # Publish drone marker
        self._publish_marker()

    def _broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        # Quaternion from yaw
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)

    def _publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'drone'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = 'package://planning/meshes/quadrotor.dae'
        marker.mesh_use_embedded_materials = True
        marker.action = Marker.ADD

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        marker.pose.orientation.z = math.sin(self.yaw / 2.0)
        marker.pose.orientation.w = math.cos(self.yaw / 2.0)

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Fallback: if mesh not found, use arrow
        marker.type = Marker.ARROW
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MockDroneTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
