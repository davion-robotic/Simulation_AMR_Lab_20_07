#!/usr/bin/env python3
"""stop_localization.py: Node ROS 2 xử lý riêng việc dừng tiến trình định vị (AMCL)."""

import json
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mybot_common.topic_names import (
    LOCALIZATION_STOP_TOPIC,
    LOCALIZATION_STATUS_TOPIC,
)
from mybot_common.command_utils import parse_command


class StopLocalizationNode(Node):
    """ROS 2 Node chuyên trách việc dừng AMCL Localization."""

    def __init__(self):
        super().__init__('stop_localization_node')

        self.subscription = self.create_subscription(
            String, LOCALIZATION_STOP_TOPIC, self.on_stop_command, 10)

        self.status_publisher = self.create_publisher(
            String, LOCALIZATION_STATUS_TOPIC, 10)

        self.get_logger().info('StopLocalizationNode ready. Waiting for stop command on /localization/stop')

    def on_stop_command(self, msg: String):
        try:
            cmd = parse_command(msg.data)
        except Exception:
            cmd = "stop"

        if cmd in ["stop", "off", "false", "0"]:
            self.stop_amcl()
        else:
            self.publish_status({"status": "error", "message": f"Unsupported command: {cmd}"})

    def stop_amcl(self):
        try:
            self.get_logger().info('Stopping Standalone AMCL process...')
            subprocess.run(['pkill', '-f', 'localization_launch'], capture_output=True, text=True)
            self.get_logger().info('Standalone AMCL process stopped.')
            self.publish_status({"status": "ok", "message": "Standalone localization stopped"})
        except Exception as e:
            self.get_logger().error(f'Failed to stop localization: {e}')
            self.publish_status({"status": "error", "message": str(e)})

    def publish_status(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.status_publisher.publish(msg)


def main():
    rclpy.init()
    node = StopLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
