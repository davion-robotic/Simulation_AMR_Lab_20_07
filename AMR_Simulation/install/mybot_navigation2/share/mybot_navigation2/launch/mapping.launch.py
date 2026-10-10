from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # ===== ARG =====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        # ===== MAPPING LIFECYCLE MANAGER =====
        # Quản lý vòng đời SLAM mapping - chỉ bắt đầu khi nhận lệnh start
        Node(
            package='mybot_mapping',
            executable='mapping_lifecycle',
            name='mapping_lifecycle_manager',
            output='screen',
        ),

        # Nhận lệnh /web/save_map_cmd và lưu map vào thư mục map của package.
        Node(
            package='mybot_mapping',
            executable='save_map',
            name='map_save_proxy',
            output='screen',
        ),
    ])
