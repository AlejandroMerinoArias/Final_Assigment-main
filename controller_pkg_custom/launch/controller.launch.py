from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your YAML config file
    pkg_share = get_package_share_directory('controller_pkg')
    default_config = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    # Optionally allow user to override config path
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to controller configuration YAML file'
    )

    command_traj_arg = DeclareLaunchArgument(
        "command_trajectory_topic",
        default_value="command/trajectory",
        description="Topic for MultiDOFJointTrajectory commands",
    )
    current_state_arg = DeclareLaunchArgument(
        "current_state_topic",
        default_value="current_state",
        description="Topic for nav_msgs/Odometry state updates",
    )

    controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ("command/trajectory", LaunchConfiguration("command_trajectory_topic")),
            ("current_state", LaunchConfiguration("current_state_topic")),
        ],
        # emulate "clear_params='true'" by forcing parameter override behavior
        emulate_tty=True
    )

    return LaunchDescription([
        config_arg,
        command_traj_arg,
        current_state_arg,
        controller_node
    ])
