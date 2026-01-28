from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="lantern_detector_pkg",
                executable="lantern_detector",
                name="lantern_detector",
                output="screen",
                parameters=[
                    {
                        "rgb_topic": "/realsense/rgb/image_rect_raw_left",
                        "depth_topic": "/realsense/depth/image_rect_raw",
                        "camera_info_topic": "/realsense/rgb/camera_info_left",
                        "detections_topic": "/lantern_detections",
                    }
                ],
            )
        ]
    )