import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # ---------------------------------------------------------
    # 1. The "Bridge": Static Transform Publisher
    # ---------------------------------------------------------
    # Connects 'true_body' to the camera frame 'Quadrotor/Sensors/DepthCamera'.
    # Translation: x=0.1m (your measurement), y=0, z=0
    # Rotation is provided as yaw/pitch/roll (radians).
    # NOTE: If your points look "sideways" in RViz, adjust the yaw here.
    body_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_camera_tf',
        arguments=['0.1', '0', '0', '-1.5708', '0', '-1.5708', 'body', 'Quadrotor/Sensors/DepthCamera']
    )

    # ---------------------------------------------------------
    # 2. The "Worker": C++ PointCloud Transformer
    # ---------------------------------------------------------
    # This runs the C++ executable you added to CMakeLists.txt.
    # It listens to /camera/depth/points, transforms them to 'world',
    # and publishes to /camera/depth/points_world.
    pointcloud_transformer_node = Node(
        package='perception',
        executable='pointcloud_transformer_node', # Must match the add_executable name in CMake
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

    return LaunchDescription([
        body_to_camera_tf,
        pointcloud_transformer_node
    ])