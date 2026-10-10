import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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
    if not os.path.exists(world_path):
        world_path = "/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_navigation2/worlds/obstacles.world"

    # Gazebo Server (Nạp đúng file world)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py"
            )
        ),
        launch_arguments={
            "world": world_path
        }.items()
    )

    # Gazebo Client (GUI 3D)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py"
            )
        )
    )

    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_bot",
            "-timeout", "120",
        ],
        output="screen"
    )

    use_rviz = LaunchConfiguration('use_rviz')

    # RViz2 Node
    rviz_config_file = os.path.join(
        package_share,
        "rviz",
        "tb3_navigation2.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz2'
        ),
        rsp,
        gzserver,
        gzclient,
        spawn_entity,
        rviz_node,
    ])