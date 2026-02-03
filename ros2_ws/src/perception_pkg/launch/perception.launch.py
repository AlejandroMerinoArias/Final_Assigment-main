from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

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

    container = ComposableNodeContainer(
        name='depth_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[depth_processing_node],
        output='screen'
    )

    return LaunchDescription([container])