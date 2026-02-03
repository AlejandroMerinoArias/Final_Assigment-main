from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch argument (currently not used further, but kept for parity)
    mav_name_arg = DeclareLaunchArgument(
        "mav_name",
        default_value="firefly",
        description="Name of the MAV"
    )
    mav_name = LaunchConfiguration("mav_name")
    odom_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/current_state_est",
        description="Odometry topic for the waypoint planner",
    )

    # Path to trajectory_config.yaml
    trajectory_config = PathJoinSubstitution([
        FindPackageShare("trajectory_generation"),
        "config",
        "trajectory_config.yaml"
    ])

    # Trajectory planner node
    planner_node = Node(
        package="trajectory_generation",
        executable="trajectory_generation_node",
        name="trajectory_generation",
        output="screen",
        parameters=[trajectory_config],
        remappings=[
            ("odom", LaunchConfiguration("odom_topic")),
        ],
    )

    waypoint_script = PathJoinSubstitution([
        FindPackageShare("trajectory_generation"),
        "scripts",
        "publish_waypoints_path.py",
    ])

    waypoint_publisher_node = ExecuteProcess(
        cmd=[
            "python3",
            waypoint_script,
            "--ros-args",
            "-p", "frame_id:=world",
            "-p", "path_topic:=waypoints",
            "-p", "publish_interval_s:=0.5",
            "-p", "max_publish_count:=1",
            "-p", "require_subscriber:=true",
        ],
        output="screen",
    )

    return LaunchDescription([
        mav_name_arg,
        odom_arg,
        planner_node,
        waypoint_publisher_node,
    ])