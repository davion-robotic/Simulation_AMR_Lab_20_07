from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mybot_navigation2',
            executable='Depht.py',
            name='depth_node',
            output='screen'
        )
    ])