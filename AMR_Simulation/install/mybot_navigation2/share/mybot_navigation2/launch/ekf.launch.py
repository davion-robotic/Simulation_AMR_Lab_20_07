from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=["/home/warlord/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/ekf.yaml",
                    {"use_sim_time": True}]
    )

    return LaunchDescription([ekf])