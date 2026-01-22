from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def _depth_to_meters(depth_image: np.ndarray) -> np.ndarray:
    if depth_image.dtype == np.uint16:
        return depth_image.astype(np.float32) * 0.001
    if depth_image.dtype == np.float32 or depth_image.dtype == np.float64:
        return depth_image.astype(np.float32)
    return depth_image.astype(np.float32)


def _median_depth(depth_m: np.ndarray, u: int, v: int, half_window: int) -> float:
    height, width = depth_m.shape[:2]
    u_min = max(0, u - half_window)
    u_max = min(width - 1, u + half_window)
    v_min = max(0, v - half_window)
    v_max = min(height - 1, v + half_window)
    patch = depth_m[v_min : v_max + 1, u_min : u_max + 1]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return float("nan")
    return float(np.median(valid))


def _pixel_to_camera(u: int, v: int, depth: float, camera_info: CameraInfo) -> Tuple[float, float, float]:
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    return x, y, depth


def _extract_centroids(mask: np.ndarray, min_area: float) -> List[Tuple[int, int]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroids: List[Tuple[int, int]] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0.0:
            continue
        u = int(moments["m10"] / moments["m00"])
        v = int(moments["m01"] / moments["m00"])
        centroids.append((u, v))
    return centroids


class LanternDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("lantern_detector")

        self.declare_parameter("rgb_topic", "/realsense/rgb/image_rect_raw_left")
        self.declare_parameter("depth_topic", "/realsense/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/realsense/rgb/camera_info_left")
        self.declare_parameter("detections_topic", "/lantern_detections")
        self.declare_parameter("min_area", 8.0)
        self.declare_parameter("depth_window", 2)
        self.declare_parameter("hsv_lower", [20, 120, 120])
        self.declare_parameter("hsv_upper", [40, 255, 255])

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value

        self.min_area = float(self.get_parameter("min_area").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        hsv_lower = self.get_parameter("hsv_lower").get_parameter_value().integer_array_value
        hsv_upper = self.get_parameter("hsv_upper").get_parameter_value().integer_array_value
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)

        self.bridge = CvBridge()

        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.info_sub = Subscriber(self, CameraInfo, info_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.on_sync)

        self.pose_pub = self.create_publisher(PoseArray, detections_topic, 10)

        self.get_logger().info(
            "Lantern detector listening to rgb='%s', depth='%s', info='%s'",
            rgb_topic,
            depth_topic,
            info_topic,
        )

    def on_sync(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert images: {exc}")
            return

        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8))

        centroids = _extract_centroids(mask, self.min_area)
        if not centroids:
            self.pose_pub.publish(self._empty_pose_array(info_msg))
            return

        depth_m = _depth_to_meters(depth_image)

        poses: List[Pose] = []
        for u, v in centroids:
            depth = _median_depth(depth_m, u, v, self.depth_window)
            if not np.isfinite(depth) or depth <= 0.0:
                continue
            x, y, z = _pixel_to_camera(u, v, depth, info_msg)
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.w = 1.0
            poses.append(pose)

        pose_array = PoseArray()
        pose_array.header.stamp = rgb_msg.header.stamp
        pose_array.header.frame_id = info_msg.header.frame_id
        pose_array.poses = poses
        self.pose_pub.publish(pose_array)

    def _empty_pose_array(self, info_msg: CameraInfo) -> PoseArray:
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = info_msg.header.frame_id
        return pose_array


def main() -> None:
    rclpy.init()
    node = LanternDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()