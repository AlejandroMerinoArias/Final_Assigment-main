from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Launch argument (currently not used further, but kept for parity)
    mav_name_arg = DeclareLaunchArgument(
        "mav_name",
        default_value="firefly",
        description="Name of the MAV"
    )
    mav_name = LaunchConfiguration("mav_name")

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="current_state",
        description="Odometry topic published by the simulator/controller"
    )
    odom_topic = LaunchConfiguration("odom_topic")

    use_controller_arg = DeclareLaunchArgument(
        "use_controller",
        default_value="false",
        description="Launch the controller node alongside the mission"
    )
    use_controller = LaunchConfiguration("use_controller")

    # Path to trajectory_config.yaml
    trajectory_config = PathJoinSubstitution([
        FindPackageShare("basic_waypoint_pkg"),
        "config",
        "trajectory_config.yaml"
    ])

    # Trajectory planner node
    planner_node = Node(
        package="basic_waypoint_pkg",
        executable="basic_waypoint_node",   # ROS1: type="basic_waypoint_pkg"
        name="planner",
        output="screen",
        parameters=[trajectory_config],
        remappings=[
            ("odom", odom_topic),
        ],
    )

    # Trajectory sampler node
    sampler_node = Node(
        package="mav_trajectory_generation",
        executable="trajectory_sampler_node",
        name="sampler",
        output="screen",
        remappings=[
            ("path_segments_4D", "trajectory"),
        ],
    )

    controller_node = Node(
        package="controller_pkg",
        executable="controller_node",
        name="controller_node",
        output="screen",
        condition=IfCondition(use_controller),
    )



    return LaunchDescription([
        mav_name_arg,
        odom_topic_arg,
        use_controller_arg,
        planner_node,
        sampler_node,
        controller_node,
    ])
