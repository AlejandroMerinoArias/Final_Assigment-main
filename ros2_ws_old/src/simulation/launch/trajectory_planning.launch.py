from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trajectory_config = PathJoinSubstitution([
        FindPackageShare("basic_waypoint_pkg"),
        "config",
        "trajectory_config.yaml",
    ])

    config_arg = DeclareLaunchArgument(
        "trajectory_config",
        default_value=trajectory_config,
        description="Path to the waypoint trajectory YAML file",
    )
    odom_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/current_state_est",
        description="Odometry topic for the waypoint planner",
    )

    planner_node = Node(
        package="basic_waypoint_pkg",
        executable="basic_waypoint_node",
        name="planner",
        output="screen",
        parameters=[LaunchConfiguration("trajectory_config")],
        remappings=[
            ("odom", LaunchConfiguration("odom_topic")),
        ],
    )

    sampler_node = Node(
        package="mav_trajectory_generation",
        executable="trajectory_sampler_node",
        name="sampler",
        output="screen",
        remappings=[
            ("path_segments_4D", "trajectory"),
        ],
    )

    return LaunchDescription([
        config_arg,
        odom_arg,
        planner_node,
        sampler_node,
    ])