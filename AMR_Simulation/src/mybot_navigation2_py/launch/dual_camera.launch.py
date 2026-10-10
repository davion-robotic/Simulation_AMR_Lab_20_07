from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource


ASTRA_LAUNCH = '/home/amr/astra_ws/ros2_astra_camera/astra_camera/launch/astra.launch.xml'


def generate_launch_description():
    # Clean up stale semaphore from any previous crashed session.
    # The package uses /dev/shm/sem.astra_device_sem to serialize USB opens —
    # if a previous run crashed while holding it, every new launch will fail
    # with "Resource busy" until this is cleared.
    cleanup = ExecuteProcess(
        cmd=['bash', '-c', 'rm -f /dev/shm/sem.astra_device_sem /dev/shm/astra_device_*'],
        output='screen'
    )

    cam1 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(ASTRA_LAUNCH),
        launch_arguments={
            'camera_name':      'camera_01',
            'serial_number':    "'19030530319'",
            'device_num':       '2',
            'enable_color':     'false',  # Astra Mini S has no UVC color camera
            'enable_depth':     'true',
            'enable_ir':        'false',
            'connection_delay': '500',
            'publish_tf': 'false',
        }.items()
    )

    cam2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(ASTRA_LAUNCH),
        launch_arguments={
            'camera_name':      'camera_02',
            'serial_number':    "'19030530142'",
            'device_num':       '2',
            'enable_color':     'false',  # Astra Mini S has no UVC color camera
            'enable_depth':     'true',
            'enable_ir':        'false',
            'connection_delay': '500',
            'publish_tf': 'false',
        }.items()
    )

    return LaunchDescription([
        cleanup,
        cam1,
        cam2,
    ])
