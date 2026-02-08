#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Launch the mission components: Perception, FSM, and RViz."""

    # RViz config file
    rviz_config = PathJoinSubstitution([
        FindPackageShare("fsm"), "mission.rviz"
    ])

    # Include perception launch (depth -> pointcloud)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("perception"), "launch", "perception.launch.py"])
        ),
    )

    # Mission FSM node
    fsm_node = Node(
        package='fsm',
        executable='mission_fsm_node',
        name='mission_fsm_node',
        output='screen',
        parameters=[
            # Parameters can be added here
            # {'takeoff_altitude': 2.0},
        ],
    )

    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        perception_launch,
        fsm_node,
        rviz_node,
    ])
