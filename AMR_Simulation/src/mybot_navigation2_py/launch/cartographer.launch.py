from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CARTOGRAPHER_CONFIG_DIR = '/home/amr/my_bot/src/mybot_navigation2_py/param'


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # ===== ARG =====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        # ===== CARTOGRAPHER SLAM =====
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', CARTOGRAPHER_CONFIG_DIR,
                '-configuration_basename', 'cartographer.lua',
            ],
            remappings=[
                ('scan', '/scan_filtered'),
            ]
        ),

        # ===== OCCUPANCY GRID (publishes /map for RViz / map saving) =====
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
                'publish_period_sec': 1.0,
            }]
        ),

        # ===== FORCE MOTORS DISABLED ON LAUNCH =====
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub',
                '--rate', '1.0',
                '/motor_enable',
                'std_msgs/msg/Bool',
                '{data: false}',
            ],
            output='screen',
        ),
    ])
