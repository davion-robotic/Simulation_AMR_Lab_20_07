from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/home/warlord/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/Depht.py'],
            output='screen'
        )
    ])