from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    collision_params = "/home/amr/my_bot/src/mybot_navigation2_py/param/Collision.yaml"

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_params]
    )
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'smoothing_frequency': 10.0,
            'scale_velocities': False,
            'feedback': 'OPEN_LOOP',
            'max_velocity': [1.2, 0.0, 6.0],
            'min_velocity': [0.0, 0.0, -6.0],
            'deadband_velocity': [0.0, 0.0, 0.0],
            'velocity_timeout': 1.0,
            'max_accel': [0.6, 0.0, 3.2],
            'max_decel': [-0.6, 0.0, -3.2],
            'odom_topic': '/odom',
            'enable_stamped_cmd_vel': False
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel_out'),
            ('cmd_vel_smoothed', '/cmd_vel_control')
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['collision_monitor',
                           'velocity_smoother',]
        }]
    )


    return LaunchDescription([
        velocity_smoother_node,
        collision_monitor_node,
        lifecycle_manager
    ])