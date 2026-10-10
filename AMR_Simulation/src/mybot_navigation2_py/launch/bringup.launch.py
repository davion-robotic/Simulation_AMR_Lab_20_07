from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ===== Launch args =====
    serial_port     = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id        = LaunchConfiguration('frame_id')

    # ===== URDF =====
    robot_description = ParameterValue(
        Command(['xacro ', '/home/amr/my_bot/src/mybot_navigation2/description/robot.urdf.xacro']),
        value_type=str
    )

    return LaunchDescription([

        # =========================
        # ARGS
        # =========================
        DeclareLaunchArgument('serial_port',     default_value='/dev/lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id',        default_value='laser_frame'),

        # =========================
        # 🔥 DUAL ASTRA CAMERAS
        # =========================
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                '/home/amr/my_bot/src/mybot_navigation2_py/launch/dual_camera.launch.py'
            )
        ),

        # =========================
        # 🔥 CAMERA TF
        # =========================
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='cam01_link_to_depth',
             arguments=['0','0','0','0','0','0','camera_01_link','camera_01_depth_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='cam01_depth_to_optical',
             arguments=['0','0','0','-1.5708','0','-1.5708',
                        'camera_01_depth_frame','camera_01_depth_optical_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='cam02_link_to_depth',
             arguments=['0','0','0','0','0','0','camera_02_link','camera_02_depth_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='cam02_depth_to_optical',
             arguments=['0','0','0','-1.5708','0','-1.5708',
                        'camera_02_depth_frame','camera_02_depth_optical_frame']),

        # =========================
        # 🔥 POINTCLOUD → SCAN (camera_01)
        # =========================
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='cloud_to_scan_cam01',
            remappings=[
                ('cloud_in', '/camera_01/depth/points'),
                ('scan',     '/scan_cam01'),
            ],
            parameters=[{
                'target_frame':        'base_link',
                'transform_tolerance': 0.05,
                'min_height':          0.1,
                'max_height':          0.25,
                'angle_min':          -3.14,
                'angle_max':           3.14,
                'angle_increment':     0.01,
                'range_min':           0.1,
                'range_max':           10.0,
                'use_inf':             True,
            }],
            output='screen'
        ),

        # =========================
        # 🔥 POINTCLOUD → SCAN (camera_02)
        # =========================
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='cloud_to_scan_cam02',
            remappings=[
                ('cloud_in', '/camera_02/depth/points'),
                ('scan',     '/scan_cam02'),
            ],
            parameters=[{
                'target_frame':        'base_link',
                'transform_tolerance': 0.05,
                'min_height':          0.5,
                'max_height':          1.8,
                'angle_min':          -3.14,
                'angle_max':           3.14,
                'angle_increment':     0.01,
                'range_min':           0.1,
                'range_max':           10.0,
                'use_inf':             True,
            }],
            output='screen'
        ),

        # =========================
        # 🔥 RPLIDAR
        # =========================
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port':      serial_port,
                'serial_baudrate':  serial_baudrate,
                'frame_id':         frame_id,
                'inverted':         False,
                'angle_compensate': True,
            }]
        ),

        # =========================
        # 🔥 ROBOT STATE
        # =========================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),

        # =========================
        # 🔥 UART
        # =========================
        Node(
            package='mybot_navigation2_py',
            executable='ControlRobotOdom',
            name='uart_bridge',
            output='screen'
        ),

        # =========================
        # 🔥 SCAN FILTER
        # =========================
        Node(
            package='mybot_navigation2_py',
            executable='ScanFilter',
            name='scan_filter',
            output='screen'
        ),

        # =========================
        # 🔥 ENABLE MOTORS
        # =========================
        # Delay 3 s so uart_bridge is ready before publishing
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '--once',
                         '/motor_enable', 'std_msgs/msg/Bool', '{data: true}'],
                    output='screen'
                )
            ]
        ),
    ])
