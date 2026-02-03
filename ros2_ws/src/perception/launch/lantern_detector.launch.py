from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    detector_node = Node(
        package="perception",
        executable="lantern_detector",
        name="lantern_detector",
        output="screen",
        parameters=[
            {
                "semantic_topic": "/realsense/semantic/image_rect_raw",
                "depth_topic": "/realsense/depth/image",
                "camera_info_topic": "/realsense/semantic/camera_info",
                "world_frame": "world",
                "body_frame": "body",
                "camera_offset": [0.1, 0.0, 0.0],
                "min_area": 5.0,
                "depth_window": 5,
                "hsv_lower": [20, 70, 70],
                "hsv_upper": [70, 255, 255],
                "gating_distance": 2.0,
                "min_observations": 5,
            }
        ],
    )
    logger_node = Node(
        package="perception",
        executable="lantern_detection_logger",
        name="lantern_detection_logger",
        output="screen",
        parameters=[
            {
                "detections_topic": "/detected_lanterns",
                "output_file": "lantern_detections.txt",
            }
        ]
    )
    return LaunchDescription([detector_node, logger_node])
