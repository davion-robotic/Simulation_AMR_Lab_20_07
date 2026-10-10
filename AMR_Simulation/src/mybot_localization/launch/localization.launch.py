from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Start Localization Node
        Node(
            package='mybot_localization',
            executable='start_localization',
            name='start_localization_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Stop Localization Node
        Node(
            package='mybot_localization',
            executable='stop_localization',
            name='stop_localization_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Initial Pose Node
        Node(
            package='mybot_localization',
            executable='initial_pose',
            name='initial_pose_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Reset Pose Node
        Node(
            package='mybot_localization',
            executable='reset_pose',
            name='reset_pose_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
