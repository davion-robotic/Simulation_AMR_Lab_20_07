import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_laser_scan_matcher')
    default_config_path = os.path.join(pkg_dir, 'config', 'laser_scan_matcher.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')
    publish_tf = LaunchConfiguration('publish_tf')

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
            'scan_topic',
            default_value='/scan',
            description='Topic name for laser scan input'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Whether to publish odom->base_link TF'
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
                }
            ],
            remappings=[
                ('scan', scan_topic),
            ]
        )
    ])
