#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2

import math
import numpy as np


class CloudToScan(Node):

    def __init__(self):
        super().__init__('cloud_to_scan')

        # ===== PARAM =====
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)

        self.range_min = 0.1
        self.range_max = 5.0

        self.num_bins = int((self.angle_max - self.angle_min) / self.angle_increment)

        # memory (reduce flickering)
        self.prev_ranges = np.full(self.num_bins, np.inf, dtype=np.float32)

        # ===== CAMERA TILT FIX (IMPORTANT) =====
        self.pitch = math.radians(10.0)   # 🔧 adjust: -20 ~ -45 depending on your mount
        self.cp = math.cos(self.pitch)
        self.sp = math.sin(self.pitch)

        # ===== SUB / PUB =====
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan_mid',
            10
        )

    def transform_point(self, x, y, z):
        """
        Convert camera optical frame -> base_link approx + pitch correction
        """
        x1 = z
        y1 = -x
        z1 = -y

        # correct pitch rotation
        y_base = self.cp * y1 - self.sp * z1
        z_base = self.sp * y1 + self.cp * z1
        x_base = x1

        return x_base, y_base, z_base

    def callback(self, msg):

        ranges = np.full(self.num_bins, np.inf, dtype=np.float32)

        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):

            x, y, z = p

            x_base, y_base, z_base = self.transform_point(x, y, z)

            # planar distance
            r = math.sqrt(x_base * x_base + y_base * y_base)

            if r < self.range_min or r > self.range_max:
                continue

            angle = math.atan2(y_base, x_base)

            if angle < self.angle_min or angle > self.angle_max:
                continue

            idx = int((angle - self.angle_min) / self.angle_increment)

            if 0 <= idx < self.num_bins:
                if r < ranges[idx]:
                    ranges[idx] = r

        # ===== fill missing values (reduce blinking) =====
        mask = np.isinf(ranges)
        ranges[mask] = self.prev_ranges[mask]
        self.prev_ranges = ranges.copy()

        # ===== publish scan =====
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = "base_link"

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        scan.scan_time = 0.1
        scan.time_increment = 0.0

        scan.ranges = ranges.tolist()
        scan.intensities = []

        self.pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()