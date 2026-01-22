import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to a 3x3 rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return np.eye(3, dtype=np.float32)
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


def build_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Create a PointCloud2 message from an (N, 3) float32 array."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = points.shape[0]
    msg.is_dense = False
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = points.astype(np.float32).tobytes()
    return msg


def pointcloud2_xyz_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Extract XYZ float32 points from a PointCloud2 message."""
    field_offsets = {field.name: field.offset for field in msg.fields}
    if not all(name in field_offsets for name in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float32)

    dtype = np.dtype(
        [
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
        ]
    )
    if msg.point_step < dtype.itemsize:
        return np.zeros((0, 3), dtype=np.float32)

    data = np.frombuffer(msg.data, dtype=dtype)
    points = np.column_stack((data["x"], data["y"], data["z"]))
    return points


class MappingNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_node")

        self.declare_parameter("pointcloud_topic", "/realsense/depth/points")
        self.declare_parameter("state_topic", "/current_state_est")
        self.declare_parameter("downsample", 4)
        self.declare_parameter("max_range_m", 8.0)
        self.declare_parameter("output_topic", "mapping/points_world")

        pointcloud_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        state_topic = self.get_parameter("state_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.downsample = max(1, int(self.get_parameter("downsample").value))
        self.max_range_m = float(self.get_parameter("max_range_m").value)

        self.last_pose: Optional[Pose] = None

        self.create_subscription(Odometry, state_topic, self.on_state, 10)
        self.create_subscription(PointCloud2, pointcloud_topic, self.on_pointcloud, 10)

        self.cloud_pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(
            "Mapping node listening to pointcloud='%s', state='%s', output='%s'",
            pointcloud_topic,
            state_topic,
            output_topic,
        )

    def on_state(self, msg: Odometry) -> None:
        self.last_pose = msg.pose.pose

    def on_pointcloud(self, msg: PointCloud2) -> None:
        if self.last_pose is None:
            return

        points_cam = pointcloud2_xyz_to_numpy(msg)
        if points_cam.size == 0:
            return

        valid = (points_cam[:, 2] > 0.0) & (points_cam[:, 2] < self.max_range_m)
        points_cam = points_cam[valid]
        if points_cam.size == 0:
            return

        stride = self.downsample
        if stride > 1:
            points_cam = points_cam[::stride]

        pose = self.last_pose
        rot = quaternion_to_rotation_matrix(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        trans = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float32)
        points_world = (rot @ points_cam.T).T + trans

        cloud_msg = build_pointcloud2(points_world, "world", msg.header.stamp)
        self.cloud_pub.publish(cloud_msg)


def main() -> None:
    rclpy.init()
    node = MappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()