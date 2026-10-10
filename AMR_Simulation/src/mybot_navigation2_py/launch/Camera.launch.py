from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='orbbec_camera',
            output='screen',

            parameters=[{
                'enable_color': False,
                'enable_depth': True,
                'enable_point_cloud': True,

                # ===== FPS =====
                'depth_fps': 30,

                # ===== RESOLUTION =====
                'depth_width': 640,
                'depth_height': 480,

                # ===== FRAME =====
                'frame_id': 'camera_link',

                # ===== POINT CLOUD =====
                'point_cloud.enable': True,
                'point_cloud.filter': True,

                # ===== ALIGN =====
                'align_depth': True,
            }],

            remappings=[
                # You will use this for scan
                ('/camera/depth/color/points', '/camera/points'),
            ]
        )

    ])