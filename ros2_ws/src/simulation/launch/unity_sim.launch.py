from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the Unity binary installed by ExternalProject into share/unity_bridge/unitysim
    pkg_share = get_package_share_directory('simulation')
    unity_bin = os.path.join(pkg_share, 'Simulation.x86_64')

    vnav = ExecuteProcess(
        cmd=[unity_bin],
        output='screen'
    )

    w_to_unity = Node(
        package='simulation',
        executable='w_to_unity',
        name='w_to_unity',
        output='screen',
        # You can override the UDP params here if you like:
        # parameters=[{'ip_address': '127.0.0.1', 'port': '12346'}]
    )

    unity_ros = Node(
        package='simulation',
        executable='unity_ros',
        name='unity_ros',
        output='screen'
    )

    state_estimate_corruptor_node = Node(
        package='simulation',
        executable='state_estimate_corruptor_node',
        name='state_estimate_corruptor_node',
        output='screen'
    )

    return LaunchDescription([vnav, w_to_unity, unity_ros, state_estimate_corruptor_node])
