import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('mybot_navigation2')

    #
    # Default map
    #
    default_map = os.path.join(
        pkg_dir,
        'map',
        'my_map.yaml'
    )

    #
    # Launch arguments
    #
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rmf = LaunchConfiguration('rmf')
    params_file = LaunchConfiguration('params_file')

    #
    # Default params
    #
    default_params = os.path.join(
        pkg_dir,
        'param',
        'humble',
        'mybot2.yaml'
    )

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    return LaunchDescription([

        #
        # Map
        #
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml'
        ),

        #
        # Nav2 Params
        #
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameter file'
        ),

        #
        # Sim Time
        #
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Nav2 Bringup (Core Navigation Stack)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_launch_file_dir,
                    'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'use_composition': 'False',
            }.items(),
        ),

        # Reset Pose Node (Listens on /localization/reset_pose and publishes (0,0,0) to /initialpose)
        Node(
            package='mybot_localization',
            executable='reset_pose',
            name='reset_pose_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Initial Pose Node (Listens on /localization/initial_pose for custom pose / last pose)
        Node(
            package='mybot_localization',
            executable='initial_pose',
            name='initial_pose_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
