"""
rmf_full.launch.py
Khởi động hoàn chỉnh RMF:
1. RMF Traffic Schedule Server
2. RMF Blockade Server
3. Topology Visualizer Node (Hiển thị nét vẽ chuẩn Open-RMF màu cam)
4. Fleet Adapter Node
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('mybot_rmf')
    default_cfg = os.path.join(pkg, 'config', 'rmf_config.yaml')
    start_topology_mgr = LaunchConfiguration('start_topology_manager')

    # 1. RMF Traffic Servers (Schedule + Blockade)
    traffic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rmf_traffic.launch.py')),
    )

    # 2. Topology Visualizer Node (Bản vẽ chuẩn Open-RMF: dải màu cam, khối vuông 3D)
    viz_node = Node(
        package='mybot_rmf',
        executable='viz_navgraph',
        name='navgraph_viz',
        output='screen',
    )

    # 3. Topology Manager Node (Mặc định tắt nếu chạy chung với tablet_bridge.launch.py để tránh xung đột 2 node)
    topology_mgr_node = Node(
        package='mybot_rmf',
        executable='topology_manager',
        name='topology_manager',
        output='screen',
        condition=IfCondition(start_topology_mgr),
    )

    # 4. Fleet Adapter Node (chạy sau 3 giây)
    fleet_launch = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'rmf_fleet.launch.py')),
            launch_arguments={
                'config': LaunchConfiguration('config'),
            }.items(),
        )]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'config',
            default_value=default_cfg,
            description='Path to rmf_config.yaml',
        ),

        DeclareLaunchArgument(
            'start_topology_manager',
            default_value='false',
            description='Start topology_manager node (set true if running without tablet_bridge)',
        ),

        traffic_launch,
        viz_node,
        topology_mgr_node,
        fleet_launch,
    ])

