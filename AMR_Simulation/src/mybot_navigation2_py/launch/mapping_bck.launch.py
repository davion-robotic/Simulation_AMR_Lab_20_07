from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    SLAM Toolbox config for RPLIDAR A1.

    REFERENCES:
    - lidarbot (TheNoobInventor): https://github.com/TheNoobInventor/lidarbot
      A working RPLIDAR A1 + ROS2 Humble + RPi4 robot — uses MOSTLY DEFAULTS
    - Research paper (KARTO-SLAM tuning): https://pmc.ncbi.nlm.nih.gov/articles/PMC9506160/
      Published tuning study confirms defaults work well for small robots

    KEY INSIGHT: For RPLIDAR A1, the official defaults are CLOSE TO OPTIMAL.
    Aggressive tuning (very permissive loop closure, huge search spaces)
    creates FALSE positive loops that warp the map, not better loops.

    YOUR ADJUSTMENTS from defaults:
    - 180° front-only lidar coverage
    - Poor wheel odometry
    - Want recorrection
    """

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,

                # ── FRAMES ──────────────────────────────────────────────
                'odom_frame':  'odom',
                'base_frame':  'base_link',
                'map_frame':   'map',
                'scan_topic':  '/scan_filtered',
                'mode':        'mapping',

                # ── ROS / TF ─────────────────────────────────────────────
                # Following lidarbot defaults
                'transform_publish_period': 0.02,
                'transform_timeout':        0.2,
                'tf_buffer_duration':       30.0,
                'restamp_tf':               False,
                'use_map_saver':            True,
                'enable_interactive_mode':  True,
                'debug_logging':            False,
                'throttle_scans':           1,

                # ── MAP ──────────────────────────────────────────────────
                'resolution':          0.05,

                # lidarbot uses 5.0. We use 2.0 as compromise:
                # fast enough to see updates while mapping, but not so fast
                # it eats CPU on a large grid.
                'map_update_interval': 2.0,

                'min_laser_range':     0.02,    # match filter
                'max_laser_range':     8.0,    # A1 reliable range (not the spec 12m)

                # ── WHEN TO ADD A NEW SCAN ───────────────────────────────
                # lidarbot uses 0.5 (default).
                # 0.2 = denser for A1's slow 5.5Hz scan rate (more chances
                # for loop matches) but not so dense it bloats the graph.
                'minimum_travel_distance':           0.2,
                'minimum_travel_heading':            0.2,
                'minimum_time_interval':             0.5,
                'check_min_dist_and_heading_precisely': False,

                # ── SCAN BUFFER ──────────────────────────────────────────
                # Research paper says 30 is OPTIMAL (above 30 hurts!).
                # Don't go to 100, 200 — that creates "scan zombies" that
                # match to OLD pose estimates that have since drifted.
                # 30 scans × 0.2m = 6m of memory.
                # If your map needs longer loops, the answer is BETTER ODOM,
                # not bigger buffer.
                'scan_buffer_size':                  30,
                'scan_buffer_maximum_scan_distance': 15.0,   # 2.5× lidar range

                # ── LOCAL SCAN MATCHING ──────────────────────────────────
                'use_scan_matching':   True,
                'use_scan_barycenter': True,

                # lidarbot default 0.1. Keep at default.
                'link_match_minimum_response_fine': 0.1,

                # lidarbot default 1.5. Bump to 2.0 for slightly faster driving.
                'link_scan_maximum_distance':       2.0,

                # ── CORRELATION (use defaults — they work) ───────────────
                'correlation_search_space_dimension':       0.5,
                'correlation_search_space_resolution':      0.01,
                'correlation_search_space_smear_deviation': 0.1,

                # ── SCAN MATCHER ─────────────────────────────────────────
                # All defaults from lidarbot — they're proven
                'distance_variance_penalty':  0.5,
                'angle_variance_penalty':     1.0,
                'fine_search_angle_offset':   0.00349,
                'coarse_search_angle_offset': 0.349,
                'coarse_angle_resolution':    0.0349,
                'minimum_angle_penalty':      0.9,
                'minimum_distance_penalty':   0.5,
                'use_response_expansion':     True,
                'min_pass_through':           2,
                'occupancy_threshold':        0.1,

                # ── LOOP CLOSURE (DEFAULTS WORK — don't over-tune) ───────
                'do_loop_closing': True,

                # lidarbot default 3.0 is fine for small spaces.
                # For "bigger maps" bump to 8.0. Don't go higher — wider
                # searches find FALSE matches more than real ones.
                'loop_search_maximum_distance':       8.0,

                # KEEP DEFAULT 3.0. Higher values let bad odom estimates
                # become "valid" loop candidates → map corruption.
                'loop_match_maximum_variance_coarse': 3.0,

                # ⬇ Slightly lower than default 0.35 for 180° lidar.
                # 180° = ~half the points = lower max achievable response.
                # 0.3 is permissive enough; 0.05-0.1 (what I had before)
                # is asking for false loops.
                'loop_match_minimum_response_coarse': 0.3,

                # KEEP DEFAULT 0.45. This is the gatekeeper against
                # false loops that warp the map.
                'loop_match_minimum_response_fine':   0.45,

                # ⬇ Lower than default 10. Lidarbot uses 10 (default) and
                # it works. 6 is more practical for smaller robots without
                # being so loose it accepts noise.
                'loop_match_minimum_chain_size':      6,

                # ⬇ Default 8.0. Keep default.
                'loop_search_space_dimension':        8.0,
                'loop_search_space_resolution':       0.05,
                'loop_search_space_smear_deviation':  0.03,

                # ── SOLVER ───────────────────────────────────────────────
                'solver_plugin':            'solver_plugins::CeresSolver',
                'ceres_linear_solver':      'SPARSE_NORMAL_CHOLESKY',
                'ceres_preconditioner':     'SCHUR_JACOBI',
                'ceres_trust_strategy':     'LEVENBERG_MARQUARDT',
                'ceres_dogleg_type':        'TRADITIONAL_DOGLEG',

                # lidarbot uses None. We use HuberLoss because:
                # 1) Your odom is poor (lidarbot has IMU+EKF — far better)
                # 2) HuberLoss is the only safety net against false loops
                'ceres_loss_function':      'HuberLoss',
                'ceres_max_num_iterations': 50,    # default

                'stack_size_to_use':        40000000,
            }]
        ),

        # ===== FORCE MOTORS DISABLED ON LAUNCH =====
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub',
                '--rate', '1.0',
                '/motor_enable',
                'std_msgs/msg/Bool',
                '{data: false}',
            ],
            output='screen',
        ),
    ])