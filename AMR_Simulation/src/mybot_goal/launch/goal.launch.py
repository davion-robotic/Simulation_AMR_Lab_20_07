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

        # Create Goal Node
        Node(
            package='mybot_goal',
            executable='create_goal',
            name='create_goal_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Update Goal Node
        Node(
            package='mybot_goal',
            executable='update_goal',
            name='update_goal_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Delete Goal Node
        Node(
            package='mybot_goal',
            executable='delete_goal',
            name='delete_goal_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Get Goals Node
        Node(
            package='mybot_goal',
            executable='get_goals',
            name='get_goals_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Go Goal Node (Navigate to Goal)
        Node(
            package='mybot_goal',
            executable='go_goal',
            name='go_goal_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
