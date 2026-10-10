import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ROS Bridge Websocket (Listen on 0.0.0.0:9090)
    rosbridge_websocket = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml"
            )
        ),
        launch_arguments={
            "address": "0.0.0.0",
            "port": "9090",
        }.items()
    )

    # Robot Status Publisher Node (from mybot_status package)
    robot_status_node = Node(
        package="mybot_status",
        executable="robot_status_publisher",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    # Map Manager Node (from mybot_mapping package)
    map_manager_node = Node(
        package="mybot_mapping",
        executable="map_manager",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    # Mapping Lifecycle Manager (from mybot_mapping package)
    mapping_lifecycle_node = Node(
        package="mybot_mapping",
        executable="mapping_lifecycle",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    # Save Map Proxy (from mybot_mapping package)
    save_map_node = Node(
        package="mybot_mapping",
        executable="save_map",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    # Load Map Proxy (from mybot_mapping package)
    load_map_node = Node(
        package="mybot_mapping",
        executable="load_map",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    # Localization Manager Launch (from mybot_localization package)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mybot_localization"),
                "launch",
                "localization.launch.py"
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Goal Manager Launch (from mybot_goal package)
    goal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mybot_goal"),
                "launch",
                "goal.launch.py"
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Topology Manager Node (from mybot_rmf package)
    topology_manager_node = Node(
        package="mybot_rmf",
        executable="topology_manager",
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    return LaunchDescription([
        declare_sim_time,
        rosbridge_websocket,
        robot_status_node,
        map_manager_node,
        mapping_lifecycle_node,
        save_map_node,
        load_map_node,
        topology_manager_node,
        localization_launch,
        goal_launch,
    ])

