import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ===== CONFIG =====
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')
    params_file  = LaunchConfiguration('params_file')

    # ===== NAV2 PATH =====
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # ===== PARAM PATHS =====
    pkg_param = os.path.join(
        get_package_share_directory('mybot_navigation2_py'), 'param'
    )
    collision_stop_params = pkg_param + '/Collision_stop.yaml'
    collision_slow_params = pkg_param + '/Collision_slow.yaml'

    # ===== PIPELINE =====
    #
    #  nav2 /cmd_vel
    #        │
    #        ├──→ collision_monitor_stop ──→ /cmd_vel_stop_out
    #        │                                       │
    #        └──────────────────────────────→ stop_timer_relay
    #                                                │
    #                                         /cmd_vel_relay
    #                                                │
    #                                  collision_monitor_slow   (slowdown always active)
    #                                                │
    #                                         /cmd_vel_slowed
    #                                                │
    #                                       velocity_smoother
    #                                                │
    #                                        /cmd_vel_control → robot

    # ===== COLLISION MONITOR — STOP ZONE ONLY =====
    collision_monitor_stop = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor_stop',
        output='screen',
        parameters=[collision_stop_params],
    )

    # ===== STOP-TIMER RELAY =====
    # Gates the stop-zone signal with a 3s hold + 5s disable window.
    # Slowdown polygons are downstream and unaffected.
    stop_timer_relay_node = Node(
        package='mybot_navigation2_py',
        executable='stop_timer_relay',
        name='stop_timer_relay',
        output='screen',
        parameters=[{'use_sim_time': False}],
        # reads: /cmd_vel (nav2), /cmd_vel_stop_out (stop monitor)
        # publishes: /cmd_vel_relay
    )

    # ===== COLLISION MONITOR — SLOWDOWN ZONES ONLY =====
    collision_monitor_slow = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor_slow',
        output='screen',
        parameters=[collision_slow_params],
        # reads /cmd_vel_relay, publishes /cmd_vel_slowed
    )

    # ===== VELOCITY SMOOTHER =====
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='collision_velocity_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'smoothing_frequency': 10.0,
            'scale_velocities': False,
            'feedback': 'OPEN_LOOP',

            'max_velocity':      [1.4,  0.0,  6.0],
            'min_velocity':      [0.0,  0.0, -6.0],
            'deadband_velocity': [0.0,  0.0,  0.0],
            'velocity_timeout':  1.0,

            'max_accel': [0.7,  0.0,  6.2],
            'max_decel': [-0.7, 0.0, -6.2],

            'odom_topic': '/odom',
            'enable_stamped_cmd_vel': False,
        }],
        remappings=[
            ('cmd_vel',          '/cmd_vel_slowed'),
            ('cmd_vel_smoothed', '/cmd_vel_control'),
        ]
    )

    # ===== LIFECYCLE MANAGER (collision layer) =====
    lifecycle_manager_collision = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'collision_velocity_smoother',
                'collision_monitor_stop',
                'collision_monitor_slow',
            ],
        }]
    )

    # ===== AUTO-ENABLE MOTORS ON LAUNCH =====
    enable_motors = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '--rate', '1.0',
            '/motor_enable',
            'std_msgs/msg/Bool',
            '{data: true}',
        ],
        output='screen',
    )

    return LaunchDescription([

        # ===== ARGUMENTS =====
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('mybot_navigation2_py'), 'map', 'big_map2_edited.yaml'
            )
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('mybot_navigation2_py'), 'param', 'mybot2.yaml'
            )
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        # ===== FULL NAV2 =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                'map':             map_file,
                'params_file':     params_file,
                'use_sim_time':    use_sim_time,
                'autostart':       'true',
                'use_composition': 'False',
            }.items(),
        ),

        # ===== COLLISION LAYER =====
        collision_monitor_stop,
        stop_timer_relay_node,
        collision_monitor_slow,
        velocity_smoother_node,
        lifecycle_manager_collision,

        # ===== ENABLE MOTORS =====
        enable_motors,
    ])
