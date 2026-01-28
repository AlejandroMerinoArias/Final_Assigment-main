from pathlib import Path
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


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


def main() -> None:
    rclpy.init()
    node = LanternDetectionLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()