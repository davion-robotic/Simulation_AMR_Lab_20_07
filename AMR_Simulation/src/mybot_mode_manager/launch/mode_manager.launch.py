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
        description='Default map name for navigation mode'
    )

    mode_manager_node = Node(
        package='mybot_mode_manager',
        executable='mode_manager_node',
        name='mode_manager_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'default_map': default_map,
            'set_mode_topic': '/system_mode/set_mode',
            'state_topic': '/system_mode/state',
            'cmd_vel_topic': '/cmd_vel',
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_sim_time,
        declare_default_map,
        mode_manager_node
    ])

