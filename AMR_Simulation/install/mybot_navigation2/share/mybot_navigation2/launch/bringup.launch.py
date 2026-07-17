from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Đường dẫn tới file robot.urdf.xacro trong package
    xacro_file = os.path.join(
        get_package_share_directory('mybot_navigation2'),
        'description',
        'robot.urdf.xacro'
    )

    robot_description = ParameterValue(
    Command([
        'xacro',
        ' ',
        xacro_file
    ]),
    value_type=str
    )

    return LaunchDescription([

        # Publish robot model + TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen'
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Node chính
        Node(
            package='mybot_navigation2',
            executable='ControlRobotOdom.py',
            name='uart_bridge',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])