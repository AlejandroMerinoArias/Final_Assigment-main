from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="depth_pointcloud",
                output="screen",
                remappings=[
                    ("image_rect", "/realsense/depth/image_rect_raw"),
                    ("camera_info", "/realsense/depth/camera_info"),
                    ("points", "/realsense/depth/points"),
                ],
            ),
            Node(
                package="mapping_pkg",
                executable="mapping_node",
                name="mapping_node",
                output="screen",
                parameters=[
                    {"pointcloud_topic": "/realsense/depth/points"},
                    {"state_topic": "/current_state_est"},
                    {"downsample": 4},
                    {"max_range_m": 8.0},
                    {"output_topic": "mapping/points_world"},
                ],
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                parameters=[
                    {"resolution": 0.25},
                    {"frame_id": "world"},
                ],
                remappings=[
                    ("cloud_in", "mapping/points_world"),
                ],
            )
        ]
    )