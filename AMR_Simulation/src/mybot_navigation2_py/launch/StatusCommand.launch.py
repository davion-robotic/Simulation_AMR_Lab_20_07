from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    uartipad_node = Node(
        package='mybot_navigation2_py',
        executable='uartipad',
        name='uartipad',
        output='screen'
    )

    pointgo_node = Node(
        package='mybot_navigation2_py',
        executable='PointGo',
        name='pointgo',
        output='screen'
    )

    status_node = Node(
        package='mybot_navigation2_py',
        executable='Status',
        name='status',
        output='screen'
    )

    return LaunchDescription([
        uartipad_node,
        pointgo_node,
        status_node,
    ])
