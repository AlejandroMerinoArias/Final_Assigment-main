import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401
from message_filters import ApproximateTimeSynchronizer, Subscriber


@dataclass
class Track:
    position: np.ndarray
    observations: int
    published: bool = False


class LanternDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("lantern_detector")

        self.declare_parameter("semantic_topic", "/realsense/semantic/image_rect_raw")
        self.declare_parameter("depth_topic", "/realsense/depth/image")
        self.declare_parameter("camera_info_topic", "/realsense/semantic/camera_info")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("body_frame", "body")
        self.declare_parameter("camera_offset", [0.1, 0.0, 0.0])
        self.declare_parameter("min_area", 10.0)
        self.declare_parameter("depth_window", 5)
        self.declare_parameter("hsv_lower", [20, 90, 90])
        self.declare_parameter("hsv_upper", [70, 255, 255])
        self.declare_parameter("gating_distance", 2.0)
        self.declare_parameter("min_observations", 5)
        self.declare_parameter("tf_timeout_s", 0.2)
        self.declare_parameter("use_latest_tf_on_extrapolation", True)

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.tracks: List[Track] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detections_pub = self.create_publisher(PoseStamped, "/detected_lanterns", 10)

        semantic_sub = Subscriber(self, Image, self.get_parameter("semantic_topic").value)
        depth_sub = Subscriber(self, Image, self.get_parameter("depth_topic").value)
        info_sub = Subscriber(self, CameraInfo, self.get_parameter("camera_info_topic").value)

        sync = ApproximateTimeSynchronizer([semantic_sub, depth_sub, info_sub], 10, 0.1)
        sync.registerCallback(self.on_images)

        self.get_logger().info("Lantern detector initialized")

    def on_images(self, semantic_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        self.camera_info = info_msg
        try:
            semantic = self.bridge.imgmsg_to_cv2(semantic_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert images: {exc}")
            return

        mask = self.create_yellow_mask(semantic)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < float(self.get_parameter("min_area").value):
                continue

            centroid = self.compute_centroid(contour)
            if centroid is None:
                continue

            depth_m = self.sample_depth(depth, centroid)
            if depth_m is None or math.isnan(depth_m) or depth_m <= 0.0:
                continue

            camera_point = self.project_to_camera(centroid, depth_m)
            if camera_point is None:
                continue

            world_point = self.camera_to_world(
                camera_point,
                semantic_msg.header.stamp,
            )
            if world_point is None:
                continue

            self.update_tracks(world_point)

        self.publish_tracks(semantic_msg.header.stamp)

    def create_yellow_mask(self, image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter("hsv_lower").value, dtype=np.uint8)
        upper = np.array(self.get_parameter("hsv_upper").value, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        return mask

    @staticmethod
    def compute_centroid(contour: np.ndarray) -> Optional[Tuple[int, int]]:
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None
        u = int(moments["m10"] / moments["m00"])
        v = int(moments["m01"] / moments["m00"])
        return u, v

    def sample_depth(self, depth: np.ndarray, centroid: Tuple[int, int]) -> Optional[float]:
        u, v = centroid
        window = int(self.get_parameter("depth_window").value)
        half = max(1, window // 2)
        h, w = depth.shape[:2]
        u_min, u_max = max(0, u - half), min(w, u + half + 1)
        v_min, v_max = max(0, v - half), min(h, v + half + 1)
        patch = depth[v_min:v_max, u_min:u_max].astype(np.float32)
        if patch.size == 0:
            return None
        if depth.dtype == np.uint16:
            patch = patch / 1000.0
        median = float(np.median(patch))
        return median

    def project_to_camera(self, centroid: Tuple[int, int], depth_m: float) -> Optional[np.ndarray]:
        if self.camera_info is None:
            return None
        u, v = centroid
        k = self.camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if fx == 0 or fy == 0:
            return None
        x = (u - cx) / fx * depth_m
        y = (v - cy) / fy * depth_m
        z = depth_m
        return np.array([x, y, z], dtype=np.float64)

    def camera_to_world(self, camera_point: np.ndarray, stamp) -> Optional[np.ndarray]:
        pose_frame = self.get_parameter("body_frame").value
        offset = np.array(self.get_parameter("camera_offset").value, dtype=np.float64)
        body_point = self.camera_to_body(camera_point) + offset
        pose = PoseStamped()
        pose.header.stamp = stamp
        timeout_s = float(self.get_parameter("tf_timeout_s").value)
        use_latest = bool(self.get_parameter("use_latest_tf_on_extrapolation").value)
        pose.header.frame_id = pose_frame
        pose.pose.position.x = float(body_point[0])
        pose.pose.position.y = float(body_point[1])
        pose.pose.position.z = float(body_point[2])
        pose.pose.orientation.w = 1.0
        world_pose = self.transform_pose(pose, timeout_s, use_latest)
        if world_pose is None:
            return None

        return np.array(
            [
                world_pose.pose.position.x,
                world_pose.pose.position.y,
                world_pose.pose.position.z,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def camera_to_body(camera_point: np.ndarray) -> np.ndarray:
        rotation = np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        )
        return rotation @ camera_point

    def transform_pose(self, pose: PoseStamped, timeout_s: float, use_latest: bool) -> Optional[PoseStamped]:
        try:
             return self.tf_buffer.transform(
                pose,
                self.get_parameter("world_frame").value,
                timeout=rclpy.duration.Duration(seconds=timeout_s),
            )
        except Exception as exc:
            if use_latest and "extrapolation into the future" in str(exc).lower():
                try:
                    pose.header.stamp = rclpy.time.Time()
                    return self.tf_buffer.transform(
                        pose,
                        self.get_parameter("world_frame").value,
                        timeout=rclpy.duration.Duration(seconds=timeout_s),
                    )
                except Exception as retry_exc:
                    self.get_logger().warn(f"TF transform failed: {retry_exc}")
                    return None
            self.get_logger().warn(f"TF transform failed: {exc}")
            return None
        
    def update_tracks(self, world_point: np.ndarray) -> None:
        gating_distance = float(self.get_parameter("gating_distance").value)
        best_track = None
        best_distance = None
        for track in self.tracks:
            distance = float(np.linalg.norm(track.position - world_point))
            if distance <= gating_distance and (best_distance is None or distance < best_distance):
                best_track = track
                best_distance = distance

        if best_track is None:
            self.tracks.append(Track(position=world_point, observations=1))
            return

        count = best_track.observations
        best_track.position = (best_track.position * count + world_point) / (count + 1)
        best_track.observations += 1

    def publish_tracks(self, stamp) -> None:
        min_observations = int(self.get_parameter("min_observations").value)
        world_frame = self.get_parameter("world_frame").value
        for track in self.tracks:
            if track.observations < min_observations:
                continue
            if track.published:
                continue
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = world_frame
            pose.pose.position.x = float(track.position[0])
            pose.pose.position.y = float(track.position[1])
            pose.pose.position.z = float(track.position[2])
            pose.pose.orientation.w = 1.0
            self.detections_pub.publish(pose)
            track.published = True

def main() -> None:
    rclpy.init()
    node = LanternDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


class LanternDetectionLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("lantern_detection_logger")

        self.declare_parameter("detections_topic", "/detected_lanterns")
        self.declare_parameter("output_file", "lantern_detections.txt")

        self.detections: List[PoseStamped] = []
        self.create_subscription(
            PoseStamped,
            self.get_parameter("detections_topic").value,
            self.on_detection,
            10,
        )

        self.get_logger().info("Lantern detection logger initialized")

    def on_detection(self, msg: PoseStamped) -> None:
        self.detections.append(msg)

    def destroy_node(self) -> bool:
        self.write_detections_to_file()
        return super().destroy_node()

    def write_detections_to_file(self) -> None:
        output_file = Path(self.get_parameter("output_file").value)
        lines = ["frame_id,x,y,z"]
        for detection in self.detections:
            position = detection.pose.position
            lines.append(
                f"{detection.header.frame_id},{position.x:.3f},{position.y:.3f},{position.z:.3f}"
            )
        try:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            output_file.write_text("\n".join(lines) + "\n")
            self.get_logger().info(f"Wrote lantern detections to {output_file}")
        except Exception as exc:
            self.get_logger().warn(f"Failed to write lantern detections: {exc}")


def logger_main() -> None:
    rclpy.init()
    node = LanternDetectionLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
