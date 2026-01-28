from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="camera_recorder_pkg",
                executable="camera_recorder",
                name="camera_recorder",
                output="screen",
                parameters=[
                    {
                        "rgb_left_topic": "/realsense/rgb/image_rect_raw_left",
                        "rgb_right_topic": "/realsense/rgb/image_rect_raw_right",
                        "depth_topic": "/realsense/depth/image_rect_raw",
                        "semantic_topic": "/realsense/semantic/image_rect_raw",
                        "output_dir": "camera_videos",
                        "fps": 15.0,
                        "depth_max_m": 8.0,
                        "depth_colormap": True,
                    }
                ],
            )
        ]
    )