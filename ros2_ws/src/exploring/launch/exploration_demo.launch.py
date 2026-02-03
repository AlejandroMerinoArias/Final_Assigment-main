"""
Launch file for the Exploration Demo.

Starts:
  - exploration_manager (C++)
  - mock_map_publisher (Python)
  - mock_drone_tf (Python)
  - RViz2 with exploration config
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('exploring')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'exploration_demo.rviz')

    return LaunchDescription([
        # Exploration Manager (C++)
        Node(
            package='exploring',
            executable='exploration_manager',
            name='exploration_manager',
            output='screen',
            parameters=[
                {'min_frontier_size': 3},
                {'update_rate_hz': 2.0},
            ],
        ),

        # Mock Map Publisher (Python)
        Node(
            package='exploring',
            executable='mock_map_publisher.py',
            name='mock_map_publisher',
            output='screen',
        ),

        # Mock Drone TF (Python)
        Node(
            package='exploring',
            executable='mock_drone_tf.py',
            name='mock_drone_tf',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
