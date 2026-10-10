from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

    Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='cloud_to_scan',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',  # or base_link if you have TF
            'transform_tolerance': 0.01,

            'min_height': -0.1,
            'max_height': 0.1,

            'angle_min': -1.57,
            'angle_max': 1.57,
            'angle_increment': 0.01,

            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 5.0,

            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/camera/depth/points'),
            ('scan', '/scan'),
        ]
    )

    ])