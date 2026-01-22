from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class FusedLantern:
    position: Tuple[float, float, float]
    observations: int


class LanternFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("lantern_fusion_node")

        self.declare_parameter("detections_topic", "/lantern_detections")
        self.declare_parameter("state_topic", "/current_state_est")
        self.declare_parameter("use_state_estimate", True)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("map_topic", "/lantern_map")
        self.declare_parameter("merge_distance", 0.5)
        self.declare_parameter("min_observations", 1)
        self.declare_parameter("tf_timeout_s", 0.2)

        detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        state_topic = self.get_parameter("state_topic").get_parameter_value().string_value
        self.use_state_estimate = bool(self.get_parameter("use_state_estimate").value)
        self.world_frame = self._normalize_frame_id(
            self.get_parameter("world_frame").get_parameter_value().string_value
        )
        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.merge_distance = float(self.get_parameter("merge_distance").value)
        self.min_observations = int(self.get_parameter("min_observations").value)
        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_s").value))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_state_stamp = None
        if self.use_state_estimate:
            self.create_subscription(Odometry, state_topic, self.on_state, 10)

        self.create_subscription(PoseArray, detections_topic, self.on_detections, 10)

        self.map_pub = self.create_publisher(PoseArray, map_topic, 10)

        self.lanterns: List[FusedLantern] = []

        self.get_logger().info(
            "Lantern fusion listening to "
            f"detections='{detections_topic}', state='{state_topic}', output='{map_topic}'"
        )

    def on_state(self, msg: Odometry) -> None:
        self.last_state_stamp = msg.header.stamp

    def on_detections(self, msg: PoseArray) -> None:
        if not msg.poses:
            self.publish_map(msg.header.stamp)
            return

        lookup_stamp = msg.header.stamp
        if self.use_state_estimate and self.last_state_stamp is not None:
            msg_time = Time.from_msg(msg.header.stamp)
            state_time = Time.from_msg(self.last_state_stamp)
            lookup_stamp = msg.header.stamp if msg_time <= state_time else self.last_state_stamp
        
        source_frame = self._normalize_frame_id(msg.header.frame_id)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                source_frame,
                lookup_stamp,
                timeout=self.tf_timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(
                "Failed to transform detections from "
                f"'{source_frame}' to '{self.world_frame}': {exc}"
            )
            return
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    source_frame,
                    Time(),
                    timeout=self.tf_timeout,
                )
                self.get_logger().warn(
                    "Failed to transform detections at requested time; "
                    f"using latest TF instead: {exc}"
                )
            except TransformException as latest_exc:
                self.get_logger().warn(
                    "Failed to transform detections from "
                    f"'{source_frame}' to '{self.world_frame}': {latest_exc}"
                )
                return
            
        for pose in msg.poses:
            pose_world = do_transform_pose(pose, transform)
            if hasattr(pose_world, "pose"):
                position_ref = pose_world.pose.position
            else:
                position_ref = pose_world.position
            position = (
                float(position_ref.x),
                float(position_ref.y),
                float(position_ref.z),
            )
            self.merge_detection(position)

        self.publish_map(lookup_stamp)

    def merge_detection(self, position: Tuple[float, float, float]) -> None:
        for lantern in self.lanterns:
            if self.distance(lantern.position, position) <= self.merge_distance:
                count = lantern.observations
                lantern.position = (
                    (lantern.position[0] * count + position[0]) / (count + 1),
                    (lantern.position[1] * count + position[1]) / (count + 1),
                    (lantern.position[2] * count + position[2]) / (count + 1),
                )
                lantern.observations = count + 1
                return

        self.lanterns.append(FusedLantern(position=position, observations=1))

    @staticmethod
    def distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def publish_map(self, stamp) -> None:
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame

        poses: List[Pose] = []
        for lantern in self.lanterns:
            if lantern.observations < self.min_observations:
                continue
            pose = Pose()
            pose.position.x = lantern.position[0]
            pose.position.y = lantern.position[1]
            pose.position.z = lantern.position[2]
            pose.orientation.w = 1.0
            poses.append(pose)

        msg.poses = poses
        self.map_pub.publish(msg)

    @staticmethod
    def _normalize_frame_id(frame_id: str) -> str:
        return frame_id.lstrip("/")

def main() -> None:
    rclpy.init()
    node = LanternFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()