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
        'resolution', default_value='0.3',
        description='Size of the voxels in meters (0.3 = 30cm)'
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
        arguments=['0.1', '0', '0', '-1.5708', '0', '-1.5708', 'body', 'Quadrotor/Sensors/DepthCamera']
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
    # 3. Cloud Gate (only forwards point clouds when mapping is enabled)
    # =================================================================
    cloud_gate_node = Node(
        package='mapping_pkg',
        executable='cloud_gate',
        name='cloud_gate',
        output='screen',
    )

    # =================================================================
    # 4. OctoMap Server (receives gated point clouds)
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
            'sensor_model/max_range': 5.0,
            'latch': True,
            'filter_ground': False,
            'base_frame_id': 'body',
            'qos_overrides./cloud_in.subscription.reliability': 'best_effort'
        }],
        remappings=[
            ('cloud_in', '/gated_cloud'),
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
        parameters=[
            # Minimum distance for exploration goals (meters)
            # Goals closer than this will be rejected to avoid tiny steps
            {'min_exploration_goal_distance': 3.0},
            {'lantern_dedup_threshold': 2.5},
            # Goal-selection watchdog configuration
            # If we have not activated a new exploration goal for this many
            # seconds, consider goal selection \"stuck\" (logs only, keeps exploring).
            {'explore_goal_selection_timeout': 60.0},
            # Maximum number of consecutive failed exploration goal requests
            # before logging that goal selection appears stuck.
            {'explore_goal_selection_max_failures': 50},
            {'z_retry_max_attempts': 3},
            {'z_retry_step': 1.0},
        ],
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
    # 5b. Global Planner Node (planner goal -> waypoints path)
    # =================================================================
    global_planner_node = Node(
        package="planning",
        executable="global_planner_node",
        name="global_planner_node",
        output="screen",
        parameters=[
            {
                "path_topic": "waypoints",
                "goal_topic": "/planner/goal",
                "odom_topic": "/current_state_est",
                "octomap_topic": "/octomap_binary",
                # Safety / inflation parameters
                # Effective collision radius of the robot in meters.
                # This is the PRIMARY collision avoidance parameter — it controls
                # how close the planned flight path can get to walls/obstacles.
                # Increase this if the drone is still colliding with walls.
                "robot_radius": 1.5,
                # Collision check sampling resolution (meters)
                # Smaller = safer but slower. Should be << robot_radius.
                # Default 0.1m provides good safety/performance balance.
                "collision_check_resolution": 0.2,
                # Hard cap for each RRT* query. Prevents planner callback from
                # blocking for long periods in cluttered areas.
                "max_planning_time_sec": 1.5,
                "allow_partial_paths": False,
                "direct_path_max_distance": 25.0,
                # Allow broader vertical sampling around target goals so
                # RRT* can route through sloped cave sections.
                "z_band": 3.5,
            }
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
    # 7. Lantern Detector
    # =================================================================
    lantern_detector_node = Node(
        package="perception",
        executable="lantern_detector",
        name="lantern_detector",
        output="screen",
        parameters=[
            {
                "semantic_topic": "/realsense/semantic/image_rect_raw",
                "depth_topic": "/realsense/depth/image",
                "camera_info_topic": "/realsense/semantic/camera_info",
                "world_frame": "world",
                "body_frame": "body",
                "camera_offset": [0.1, 0.0, 0.0],
                "min_area": 5.0,
                "depth_window_scale": 0.5,
                "hsv_lower": [20, 70, 70],
                "hsv_upper": [70, 255, 255],
                "gating_distance": 2.0,
                "track_merge_distance": 5.0,
                "min_observations": 20,
            }
        ],
    )

    lantern_logger_node = Node(
        package="perception",
        executable="lantern_detection_logger",
        name="lantern_detection_logger",
        output="screen",
        parameters=[
            {
                "detections_topic": "/detected_lanterns",
                "output_file": "lantern_detections.txt",
            }
        ],
    )

    lantern_marker_node = Node(
        package="perception",
        executable="lantern_marker",
        name="lantern_marker",
        output="screen",
        parameters=[
            {
                "detections_topic": "/detected_lanterns",
                "marker_topic": "/lantern_marker",
                "sphere_diameter": 0.3,
            }
        ],
    )

    # =================================================================
    # 8. Exploration Manager (frontier-based exploration)
    # =================================================================
    exploration_manager_node = Node(
        package='exploring',
        executable='exploration_manager',
        name='exploration_manager',
        output='screen',
        parameters=[
            {'min_frontier_size': 5},
            {'update_rate_hz': 4.0}, # Faster update rate for sampling (lightweight)
            
            # Dai-Lite Sampling Parameters
            {'num_candidates': 20},           # Random samples per request
            {'downsample_grid': 1.0},         # Spatial spreading grid (m)
            {'frontier_search_radius': 10.0}, # BBX radius for frontier detection (m)
            
            # Scoring & Constraints
            {'drone_speed': 2.0},             # For travel time estimation (m/s)
            {'vertical_penalty_weight': 0.8}, # Softer altitude-change penalty
            {'vertical_penalty_deadband': 1.0}, # No penalty for small |dz| slope-following
            {'max_goal_z_delta': 3.5},       # Allow stronger altitude adaptation per goal
            {'max_goal_z_delta_stuck_bonus': 2.0}, # Extra |dz| allowance when stuck
            {'min_goal_distance': 5.0},       # Reject frontiers too close to drone (m)
            {'max_step_distance': 30.0},      # Max goal distance from drone (m)
            
            # Safety
            {'exploration_inflation_radius': 0.6},  # Goal selection only — planner's robot_radius handles path safety
            {'cave_entrance_x': -320.0},      # Actual cave entrance X coordinate
            {'min_z': -10.0},                   # Minimum navigable altitude (m)
            {'max_z': 50.0},                  # Maximum navigable altitude (m)
            {'failed_region_merge_radius': 3.0},
            {'failed_region_base_reject_radius': 2.0},
            {'failed_region_reject_radius_gain': 0.7},
            {'failed_region_max_hits': 6},
            {'backtrack_reject_distance': 2.5},
            {'backtrack_penalty_factor': 0.55},
            {'unknown_volume_weight': 0.35},
            {'unknown_volume_radius': 3.5},
            {'unknown_sample_step_multiplier': 2.0},
            {'unknown_min_cave_depth_margin': 0.5},
            {'heading_update_alpha': 0.35},
        ],
    )


    # =================================================================
    # Launch Description
    # =================================================================
    return LaunchDescription([
        # Arguments
        resolution_arg,
        # Lantern detection
        lantern_detector_node,
        lantern_logger_node,
        # Perception pipeline
        depth_container,
        body_to_camera_tf,
        pointcloud_transformer_node,
        # Mapping
        cloud_gate_node,
        octomap_node,
        # FSM & Visualization
        fsm_node,
        trajectory_generation_node,
        global_planner_node,
        lantern_marker_node,
        exploration_manager_node,
        rviz_node,
    ])
