"""
rmf_traffic.launch.py
Starts the Open-RMF schedule / traffic management servers.
Must be running before the fleet adapter.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    schedule_node = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_schedule',
        name='rmf_traffic_schedule',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    blockade_node = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_blockade',
        name='rmf_traffic_blockade',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        schedule_node,
        blockade_node,
    ])
