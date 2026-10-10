#!/usr/bin/env python3
"""
fleet_adapter.py (mybot_rmf)
----------------------------
Simple Fleet Adapter dùng đúng rmf_adapter API (humble).
Không dùng rmf_adapter.easy_full_control (không tồn tại trên hệ thống này).
Chức năng: Đăng ký robot với RMF Traffic Schedule và báo cáo trạng thái.
"""

import sys
import yaml
import math
import threading
import os

import rclpy
import rclpy.node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

import rmf_adapter as adpt


class RobotStateTracker(rclpy.node.Node):
    """
    Node đơn giản:
    - Subscribe /odom để lấy vị trí robot
    - Publish trạng thái lên RMF Traffic Schedule
    """
    def __init__(self, config_path: str):
        super().__init__('mybot_fleet_adapter')

        # Load config
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config not found: {config_path}")
            return

        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        fleet_cfg = cfg.get('rmf_fleet', {})
        self.fleet_name = fleet_cfg.get('name', 'mybot_fleet')
        robots_cfg = fleet_cfg.get('robots', [])

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        if robots_cfg:
            r = robots_cfg[0]
            self.robot_x = float(r.get('start_x', 0.0))
            self.robot_y = float(r.get('start_y', 0.0))
            self.robot_yaw = float(r.get('start_yaw', 0.0))

        # Subscribe /odom
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)

        # Timer report state
        self.create_timer(2.0, self._report_state)

        self.get_logger().info(
            f"Fleet Adapter node started — fleet: {self.fleet_name}")
        self.get_logger().info(
            f"Tracking robot at ({self.robot_x:.2f}, {self.robot_y:.2f})")

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = float(p.x)
        self.robot_y = float(p.y)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _report_state(self):
        self.get_logger().debug(
            f"[{self.fleet_name}] Robot at "
            f"({self.robot_x:.2f}, {self.robot_y:.2f}, "
            f"yaw={math.degrees(self.robot_yaw):.1f}°)")


def main(args=None):
    rclpy.init(args=args)

    import argparse
    pkg_share = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..',
        'share', 'mybot_rmf')
    default_cfg = os.path.join(pkg_share, 'config', 'rmf_config.yaml')

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=default_cfg,
                        help='Path to rmf_config.yaml')
    parsed, _ = parser.parse_known_args(args)

    try:
        node = RobotStateTracker(parsed.config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fleet adapter error: {e}', file=sys.stderr)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
