#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the mission components: Perception pipeline, Mapping, FSM, and RViz."""

    # =================================================================
    # Launch Arguments
    # =================================================================
    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='0.1',
        description='Size of the voxels in meters (0.1 = 10cm)'
    )

    # RViz config file
    rviz_config = PathJoinSubstitution([
        FindPackageShare("fsm"), "mission.rviz"
    ])

    # =================================================================
    # 1. Depth to PointCloud (from depth_to_cloud.launch.py)
    # =================================================================
    depth_processing_node = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzNode',
        name='point_cloud_xyz_node',
        remappings=[
            ('image_rect', '/realsense/depth/image'),
            ('camera_info', '/realsense/depth/camera_info'),
            ('points', '/camera/depth/points')
        ]
    )

    depth_container = ComposableNodeContainer(
        name='depth_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[depth_processing_node],
        output='screen'
    )

    # =================================================================
    # 2. PointCloud Transformer (from pointcloud_transformer.launch.py)
    # =================================================================
    # Static TF: Connect 'true_body' to the camera frame
    body_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_camera_tf',
        arguments=['0.1', '0', '0', '0', '0', '-1.5708', 'true_body', 'Quadrotor/Sensors/DepthCamera']
    )

    # C++ PointCloud Transformer: transforms points to 'world' frame
    pointcloud_transformer_node = Node(
        package='perception',
        executable='pointcloud_transformer_node',
        name='pcl_transformer',
        output='screen',
        parameters=[{
            'target_frame': 'world'
        }],
        remappings=[
            ('input', '/camera/depth/points'),
            ('output', '/camera/depth/points_world')
        ]
    )

    # =================================================================
    # 3. OctoMap Server (from mapping.launch.py)
    # =================================================================
    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': 'world',
            'use_sim_time': True,
            'sensor_model/max_range': 10.0,
            'latch': True,
            'filter_ground': False,
            'base_frame_id': 'body',
            'qos_overrides./cloud_in.subscription.reliability': 'best_effort'
        }],
        remappings=[
            ('cloud_in', '/camera/depth/points'),
            ('octomap_binary', '/octomap_binary'),
            ('occupied_cells_vis_array', '/occupied_cells_vis_array')
        ]
    )

    # =================================================================
    # 4. Mission FSM Node
    # =================================================================
    fsm_node = Node(
        package='fsm',
        executable='mission_fsm_node',
        name='mission_fsm_node',
        output='screen',
        parameters=[],
    )

    # =================================================================
    # 5. Trajectory Generation Node (waypoints -> command/trajectory)
    # =================================================================
    trajectory_config = PathJoinSubstitution([
        FindPackageShare("trajectory_generation"),
        "config",
        "trajectory_config.yaml"
    ])

    trajectory_generation_node = Node(
        package="trajectory_generation",
        executable="trajectory_generation_node",
        name="trajectory_generation",
        output="screen",
        parameters=[trajectory_config],
        remappings=[
            ("odom", "/current_state_est"),
        ],
    )

    # =================================================================
    # 6. RViz2 for Visualization
    # =================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # =================================================================
    # Launch Description
    # =================================================================
    return LaunchDescription([
        # Arguments
        resolution_arg,
        # Perception pipeline
        depth_container,
        body_to_camera_tf,
        pointcloud_transformer_node,
        # Mapping
        octomap_node,
        # FSM & Visualization
        fsm_node,
        trajectory_generation_node,
        rviz_node,
    ])
