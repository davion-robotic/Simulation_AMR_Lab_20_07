#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self._idx = None  # cached once on first message
        self.sub = self.create_subscription(LaserScan, '/scan',         self.callback, _QOS)
        self.pub = self.create_publisher(   LaserScan, '/scan_filtered', _QOS)

    def callback(self, msg: LaserScan):
        if self._idx is None:
            am, ai, n = msg.angle_min, msg.angle_increment, len(msg.ranges)
            def idx(a): return max(0, min(int((a - am) / ai), n))
            self._idx = (idx(1.570795), idx(3.14159), idx(-3.14159), idx(-1.570795))

        s1, e1, s2, e2 = self._idx
        ranges      = msg.ranges[s1:e1]      + msg.ranges[s2:e2]
        intensities = msg.intensities[s1:e1] + msg.intensities[s2:e2]
        if not ranges:
            return

        new_scan                 = LaserScan()
        new_scan.header          = msg.header  # keep original timestamp — no added latency
        new_scan.angle_min       = msg.angle_min + s1 * msg.angle_increment
        new_scan.angle_increment = msg.angle_increment
        new_scan.angle_max       = new_scan.angle_min + (len(ranges) - 1) * msg.angle_increment
        new_scan.range_min       = msg.range_min
        new_scan.range_max       = msg.range_max
        new_scan.scan_time       = msg.scan_time
        new_scan.time_increment  = msg.time_increment
        new_scan.ranges          = ranges
        new_scan.intensities     = intensities
        self.pub.publish(new_scan)

def main():
    rclpy.init()
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
