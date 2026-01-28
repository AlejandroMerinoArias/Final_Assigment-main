from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="perception_fusion_pkg",
                executable="lantern_fusion_node",
                name="lantern_fusion_node",
                output="screen",
                parameters=[
                    {
                        "detections_topic": "/lantern_detections",
                        "state_topic": "/current_state_est",
                        "use_state_estimate": True,
                        "world_frame": "world",
                        "map_topic": "/lantern_map",
                        "merge_distance": 0.5,
                        "min_observations": 1,
                        "tf_timeout_s": 0.2,
                    }
                ],
            )
        ]
    )