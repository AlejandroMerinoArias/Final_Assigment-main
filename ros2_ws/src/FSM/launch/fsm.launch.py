from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Mission FSM node for testing."""
    return LaunchDescription([
        Node(
            package='FSM',
            executable='mission_fsm_node',
            name='mission_fsm_node',
            output='screen',
            parameters=[
                # Parameters can be added here
                # {'takeoff_altitude': 2.0},
                # {'cave_entrance_x': 10.0},
            ],
        ),
    ])
