#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class WaypointPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_path_publisher")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("path_topic", "waypoints")
        self.declare_parameter("publish_interval_s", 0.5)
        self.declare_parameter("max_publish_count", 1)
        self.declare_parameter("require_subscriber", True)

        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        publish_interval = self.get_parameter("publish_interval_s").get_parameter_value().double_value
        self.max_publish_count = self.get_parameter("max_publish_count").get_parameter_value().integer_value
        self.require_subscriber = self.get_parameter("require_subscriber").get_parameter_value().bool_value

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(Path, path_topic, qos)
        self.path_msg = Path()
        self.path_msg.header.frame_id = frame_id

        waypoints = [
            (-50.0, 10.0, 10.0),
            (-100.0, 10.0, 12.0),
            (-150.0, 10.0, 14.0),
            (-200.0, 10.0, 16.0),
            (-250.0, 10.0, 18.0),
            (-320.0, 10.0, 18.0),
        ]

        # waypoints = [
        #     (-80.0, 10.0, 10.0),
        #     (-80.0, 0.0, 10.0),
        #     (-80.0, -10.0, 10.0),
        #     (-80.0, 0.0, 10.0),
        #     (-80.0, 10.0, 10.0),
        #     (-80.0, 0.0, 10.0),
        # ]

        # waypoints = [
        #     (-60.669,1.011,10.0),
        # ]

        for x, y, z in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose)

        self.publish_count = 0
        self.timer = self.create_timer(publish_interval, self.maybe_publish)

    def maybe_publish(self) -> None:
        if self.require_subscriber and self.publisher.get_subscription_count() == 0:
            return

        self.publish_path()
        self.publish_count += 1

        if self.publish_count >= self.max_publish_count:
            self.timer.cancel()

    def publish_path(self) -> None:
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now
        self.publisher.publish(self.path_msg)
        self.get_logger().info(
            f"Published waypoint path with {len(self.path_msg.poses)} poses.")


def main() -> None:
    rclpy.init()
    node = WaypointPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
