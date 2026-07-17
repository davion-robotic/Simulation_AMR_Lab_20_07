from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    websocket_client = Node(
        package="mybot_navigation2",
        executable="Websocket_Client.py",
        name="websocket_client",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        websocket_client,
    ])