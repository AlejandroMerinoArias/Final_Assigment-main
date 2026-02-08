import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Define Arguments (Variables your teammates can change)
    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='0.1',
        description='Size of the voxels in meters (0.1 = 10cm)'
    )
    
    # 2. Define the Node
    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': 'world',
            'use_sim_time': True,             # FIX 1: Sync Time
            'sensor_model/max_range': 10.0,
            'latch': True,
            'filter_ground': False,
            'base_frame_id': 'body',
            'qos_overrides./cloud_in.subscription.reliability': 'best_effort' # FIX 2: Accept Camera Data
        }],
        remappings=[
            # INPUT: Connect to Perception Node
            ('cloud_in', '/camera/depth/points'),
            
            # OUTPUT: Map for the Planner
            ('octomap_binary', '/octomap_binary'),
            
            # VISUALIZATION
            ('occupied_cells_vis_array', '/occupied_cells_vis_array')
        ]
    )

    return LaunchDescription([
        resolution_arg,
        octomap_node
    ])