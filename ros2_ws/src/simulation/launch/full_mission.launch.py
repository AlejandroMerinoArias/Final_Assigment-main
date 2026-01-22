#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("load_params", default_value="true"),
        DeclareLaunchArgument("corrupt_state_estimate", default_value="true"),
        DeclareLaunchArgument("right_image_topic", default_value="/realsense/rgb/image_rect_raw_right"),
        DeclareLaunchArgument("right_info_topic", default_value="/realsense/rgb/camera_info_right"),
        DeclareLaunchArgument("left_image_topic", default_value="/realsense/rgb/image_rect_raw_left"),
        DeclareLaunchArgument("left_info_topic", default_value="/realsense/rgb/camera_info_left"),
        DeclareLaunchArgument("depth_image_topic", default_value="/realsense/depth/image_rect_raw"),
        DeclareLaunchArgument("depth_info_topic", default_value="/realsense/depth/camera_info"),
        DeclareLaunchArgument("imu_topic", default_value="/interpolate_imu/imu"),
        DeclareLaunchArgument("command_trajectory_topic", default_value="command/trajectory"),
        DeclareLaunchArgument("current_state_topic", default_value="current_state"),
        DeclareLaunchArgument("trajectory_config", default_value=PathJoinSubstitution([
            FindPackageShare("basic_waypoint_pkg"),
            "config",
            "trajectory_config.yaml",
        ])),
        DeclareLaunchArgument("odom_topic", default_value="/current_state_est"),
    ]

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("simulation"), "launch", "simulation.launch.py"])
        ),
        launch_arguments={
            "load_params": LaunchConfiguration("load_params"),
            "corrupt_state_estimate": LaunchConfiguration("corrupt_state_estimate"),
            "right_image_topic": LaunchConfiguration("right_image_topic"),
            "right_info_topic": LaunchConfiguration("right_info_topic"),
            "left_image_topic": LaunchConfiguration("left_image_topic"),
            "left_info_topic": LaunchConfiguration("left_info_topic"),
            "depth_image_topic": LaunchConfiguration("depth_image_topic"),
            "depth_info_topic": LaunchConfiguration("depth_info_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "command_trajectory_topic": LaunchConfiguration("command_trajectory_topic"),
            "current_state_topic": LaunchConfiguration("current_state_topic"),
        }.items(),
    )

    trajectory_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("simulation"),
                "launch",
                "trajectory_planning.launch.py",
            ])
        ),
        launch_arguments={
            "trajectory_config": LaunchConfiguration("trajectory_config"),
            "odom_topic": LaunchConfiguration("odom_topic"),
        }.items(),
    )

    return LaunchDescription(declared_args + [simulation_launch, trajectory_launch])