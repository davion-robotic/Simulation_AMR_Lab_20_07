from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    task_controller_node = Node(
        package='mybot_task_controller',
        executable='task_controller_node',
        name='task_controller_node',
        output='screen',
        parameters=[{
            'cmd_topic': '/task_controller/command',
            'status_topic': '/task_controller/status',
            'cmd_vel_topic': '/cmd_vel',
            'nav2_action_name': 'navigate_to_pose'
        }]
    )

    return LaunchDescription([
        task_controller_node
    ])
