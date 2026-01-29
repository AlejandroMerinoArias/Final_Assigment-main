import os
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


@dataclass
class VideoStream:
    name: str
    writer: Optional[cv2.VideoWriter] = None
    size: Optional[tuple] = None


class CameraRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_recorder")

        self.declare_parameter("rgb_left_topic", "/realsense/rgb/left_image_raw")
        self.declare_parameter("rgb_right_topic", "/realsense/rgb/right_image_raw")
        self.declare_parameter("depth_topic", "/realsense/depth/image")
        self.declare_parameter("semantic_topic", "/realsense/semantic/image_rect_raw")
        self.declare_parameter("output_dir", "camera_videos")
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("depth_max_m", 8.0)
        self.declare_parameter("depth_colormap", True)

        self.bridge = CvBridge()
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.fps = float(self.get_parameter("fps").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.depth_colormap = bool(self.get_parameter("depth_colormap").value)

        os.makedirs(self.output_dir, exist_ok=True)

        self.streams = {
            "rgb_left": VideoStream("rgb_left"),
            "rgb_right": VideoStream("rgb_right"),
            "depth": VideoStream("depth"),
            "semantic": VideoStream("semantic"),
        }

        self.create_subscription(
            Image,
            self.get_parameter("rgb_left_topic").get_parameter_value().string_value,
            lambda msg: self.on_rgb(msg, "rgb_left"),
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("rgb_right_topic").get_parameter_value().string_value,
            lambda msg: self.on_rgb(msg, "rgb_right"),
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("semantic_topic").get_parameter_value().string_value,
            lambda msg: self.on_rgb(msg, "semantic"),
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").get_parameter_value().string_value,
            self.on_depth,
            10,
        )

        self.get_logger().info(
            f"Camera recorder writing videos to '{self.output_dir}' at {self.fps:.1f} FPS"
        )

    def on_rgb(self, msg: Image, key: str) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn("Failed to convert %s image: %s", key, exc)
            return

        self.write_frame(key, cv_image)

    def on_depth(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
        except Exception as exc:
            self.get_logger().warn("Failed to convert depth image: %s", exc)
            return

        depth_m = self.to_depth_meters(cv_image)
        depth_m = np.clip(depth_m, 0.0, self.depth_max_m)
        depth_norm = (depth_m / self.depth_max_m * 255.0).astype(np.uint8)

        if self.depth_colormap:
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
        else:
            depth_color = cv2.cvtColor(depth_norm, cv2.COLOR_GRAY2BGR)

        self.write_frame("depth", depth_color)

    def to_depth_meters(self, cv_image: np.ndarray) -> np.ndarray:
        if cv_image.dtype == np.uint16:
            return cv_image.astype(np.float32) / 1000.0
        if cv_image.dtype == np.float32 or cv_image.dtype == np.float64:
            return cv_image.astype(np.float32)
        return cv_image.astype(np.float32)

    def write_frame(self, key: str, frame: np.ndarray) -> None:
        stream = self.streams[key]
        if stream.writer is None:
            height, width = frame.shape[:2]
            stream.size = (width, height)
            filename = os.path.join(self.output_dir, f"{key}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            stream.writer = cv2.VideoWriter(filename, fourcc, self.fps, stream.size)
            self.get_logger().info(f"Opened video writer for {key} -> {filename}")

        if frame.shape[1::-1] != stream.size:
            frame = cv2.resize(frame, stream.size)

        stream.writer.write(frame)

    def destroy_node(self) -> bool:
        for stream in self.streams.values():
            if stream.writer is not None:
                stream.writer.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CameraRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()