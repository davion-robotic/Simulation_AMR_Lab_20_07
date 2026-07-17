#!/usr/bin/env python3
"""
Dispatch patrol task đến RMF qua /task_api_requests (ApiRequest JSON).
Cách dùng:
  python3 send_patrol.py --places start wp4 --loops 2
"""

import argparse
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rmf_task_msgs.msg import ApiRequest


class PatrolDispatcher(Node):

    def __init__(self):
        super().__init__('patrol_dispatcher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self._pub = self.create_publisher(ApiRequest, '/task_api_requests', qos)

    def send_patrol(self, places: list, rounds: int):
        req = ApiRequest()
        req.request_id = f"patrol_{uuid.uuid4().hex[:8]}"
        req.json_msg = json.dumps({
            "type": "dispatch_task_request",
            "request": {
                "unix_millis_earliest_start_time": 0,
                "priority": {"type": "binary", "value": 0},
                "category": "patrol",
                "description": {
                    "places": places,
                    "rounds": rounds
                },
                "requester": "patrol_dispatcher"
            }
        })
        self.get_logger().info(
            f"Sending patrol: {places} x{rounds}")
        self.get_logger().info(f"Request ID: {req.request_id}")
        self._pub.publish(req)


def main():
    parser = argparse.ArgumentParser(description='Send patrol task to RMF')
    parser.add_argument('--places', nargs='+', default=['start', 'wp4'],
                        help='Ordered list of waypoint names (e.g. start wp2 wp4)')
    parser.add_argument('--loops', type=int, default=1,
                        help='Number of patrol rounds')
    args = parser.parse_args()

    rclpy.init()
    node = PatrolDispatcher()

    time.sleep(1.0)
    node.send_patrol(args.places, args.loops)
    rclpy.spin_once(node, timeout_sec=2.0)

    rclpy.shutdown()
    print("Done. Watch /fleet_states for robot movement.")
    print(f"Monitor: ros2 topic echo /fleet_states")


if __name__ == '__main__':
    main()
