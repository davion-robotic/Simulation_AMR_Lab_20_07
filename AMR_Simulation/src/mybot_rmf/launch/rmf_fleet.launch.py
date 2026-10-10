"""
rmf_fleet.launch.py
Starts the mybot fleet adapter (RMF ↔ nav2 bridge).
Requires rmf_traffic.launch.py to be running first.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('mybot_rmf')
    default_cfg = os.path.join(pkg, 'config', 'rmf_config.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'config',
            default_value=default_cfg,
            description='Path to rmf_config.yaml',
        ),

        Node(
            package='mybot_rmf',
            executable='fleet_adapter',
            name='mybot_fleet_adapter',
            output='screen',
            arguments=['--config', LaunchConfiguration('config')],
            parameters=[{'use_sim_time': False}],
        ),
    ])
