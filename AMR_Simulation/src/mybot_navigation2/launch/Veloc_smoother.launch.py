from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'smoothing_frequency': 20.0,
            'scale_velocities': False,
            'feedback': 'OPEN_LOOP',
            'max_velocity': [0.5, 0.0, 2.5],
            'min_velocity': [-0.5, 0.0, -2.5],
            'deadband_velocity': [0.0, 0.0, 0.0],
            'velocity_timeout': 1.0,
            'max_accel': [2.5, 0.0, 3.2],
            'max_decel': [-2.5, 0.0, -3.2],
            'odom_topic': '/odom',
            'enable_stamped_cmd_vel': False
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed')
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_velocity',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['velocity_smoother']
        }]
    )

    return LaunchDescription([
        velocity_smoother_node,
        lifecycle_manager
    ])