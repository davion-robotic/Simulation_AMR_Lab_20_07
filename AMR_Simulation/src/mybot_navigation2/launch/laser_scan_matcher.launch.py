import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_scan_matcher = get_package_share_directory('ros2_laser_scan_matcher')
    default_config_path = os.path.join(pkg_scan_matcher, 'config', 'laser_scan_matcher.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_odom = LaunchConfiguration('publish_odom')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config_path,
            description='Full path to laser_scan_matcher param file'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Whether to publish odom->base_link TF'
        ),
        DeclareLaunchArgument(
            'publish_odom',
            default_value='/scan_odom',
            description='Topic name for published odometry'
        ),

        Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'publish_tf': publish_tf,
                    'publish_odom': publish_odom,
                }
            ],
            remappings=[
                ('scan', '/scan'),
            ]
        )
    ])
