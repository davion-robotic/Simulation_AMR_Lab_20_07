import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_map = LaunchConfiguration('default_map')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_default_map = DeclareLaunchArgument(
        'default_map',
        default_value='map_lab20',
        description='Default map for waypoint resolution'
    )

    task_controller_node = Node(
        package='mybot_task_controller',
        executable='task_controller_node',
        name='task_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'default_map': default_map,
            'cmd_topic': '/task_controller/command',
            'status_topic': '/task_controller/status',
            'cmd_vel_topic': '/cmd_vel',
            'nav2_action_name': 'navigate_to_pose',
            'battery_low_threshold': 20.0,
            'max_retries_per_wp': 2
        }]
    )

    return LaunchDescription([
        declare_sim_time,
        declare_default_map,
        task_controller_node
    ])
