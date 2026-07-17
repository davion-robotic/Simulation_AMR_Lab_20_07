import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = "mybot_navigation2"
    package_share = get_package_share_directory(package_name)

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "rsp.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true"
        }.items()
    )

    # World file
    world_path = os.path.join(
        package_share,
        "worlds",
        "obstacles.world"
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": world_path
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_bot",
        ],
        output="screen"
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])