#!/usr/bin/env python3
"""start_localization.py: Node ROS 2 xử lý riêng việc khởi chạy tiến trình định vị độc lập (Standalone AMCL + Map Server)."""

import json
import os
import subprocess
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mybot_common.constants import MAPS_DIR
from mybot_common.topic_names import (
    LOCALIZATION_START_TOPIC,
    LOCALIZATION_STOP_TOPIC,
    LOCALIZATION_STATUS_TOPIC,
)
from mybot_common.command_utils import parse_payload_dict, sanitize_map_name


class StartLocalizationNode(Node):
    """ROS 2 Node chuyên trách việc khởi chạy Standalone AMCL Localization (Không kèm Nav2 planner/controller)."""

    def __init__(self):
        super().__init__('start_localization_node')
        self.amcl_process = None
        self.lock = threading.Lock()

        self.subscription_start = self.create_subscription(
            String, LOCALIZATION_START_TOPIC, self.on_start_command, 10)

        self.subscription_stop = self.create_subscription(
            String, LOCALIZATION_STOP_TOPIC, self.on_stop_command, 10)

        self.status_publisher = self.create_publisher(
            String, LOCALIZATION_STATUS_TOPIC, 10)

        self.get_logger().info('StartLocalizationNode ready. Waiting for start/stop command on /localization/start')

    def on_stop_command(self, msg: String):
        self.stop_amcl()

    def stop_amcl(self):
        with self.lock:
            if self.amcl_process is not None:
                try:
                    self.get_logger().info('Stopping spawned standalone AMCL process...')
                    self.amcl_process.terminate()
                    self.amcl_process.wait(timeout=2)
                except Exception:
                    pass
                self.amcl_process = None
                self.get_logger().info('Standalone AMCL process terminated.')

    def on_start_command(self, msg: String):
        payload = parse_payload_dict(msg.data)
        cmd = str(payload.get("command", payload.get("data", "start"))).lower().strip()
        map_name = sanitize_map_name(payload.get("map_name", "map_lab20")) or "map_lab20"
        use_sim_time = payload.get("use_sim_time", True)

        if cmd in ["start", "on", "true", "1", "map_lab20"]:
            self.start_amcl(map_name, use_sim_time)
        else:
            self.publish_status({"status": "error", "message": f"Unsupported command: {cmd}"})

    def start_amcl(self, map_name: str, use_sim_time: bool = True):
        with self.lock:
            if self.amcl_process is not None and self.amcl_process.poll() is None:
                self.get_logger().warning('Standalone AMCL Localization already running.')
                self.publish_status({"status": "warning", "message": "Localization already running"})
                return

            map_path = MAPS_DIR / f"{map_name}.yaml"
            if not map_path.is_file():
                # Fallback to my_map.yaml if map_name.yaml doesn't exist
                map_path = MAPS_DIR / "my_map.yaml"

            use_sim_str = "true" if use_sim_time else "false"

            try:
                from ament_index_python.packages import get_package_share_directory
                params_file = Path(get_package_share_directory('mybot_navigation2')) / 'param' / 'humble' / 'mybot2.yaml'
            except Exception:
                params_file = None

            cmd_args = [
                'ros2', 'launch', 'nav2_bringup', 'localization_launch.py',
                f'map:={map_path}',
                f'use_sim_time:={use_sim_str}',
                'use_composition:=false'
            ]
            if params_file and params_file.is_file():
                cmd_args.append(f'params_file:={params_file}')

            try:
                self.get_logger().info(f'Starting Standalone AMCL Localization process with map {map_path}...')
                self.amcl_process = subprocess.Popen(
                    cmd_args,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                self.get_logger().info(f'Standalone AMCL Localization process started with PID {self.amcl_process.pid}')
                self.publish_status({
                    "status": "ok",
                    "action": "start_localization",
                    "map": map_name,
                    "pid": self.amcl_process.pid
                })
            except Exception as e:
                self.get_logger().error(f'Failed to start standalone localization: {e}')
                self.amcl_process = None
                self.publish_status({"status": "error", "message": str(e)})

    def publish_status(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.status_publisher.publish(msg)

    def cleanup(self):
        with self.lock:
            if self.amcl_process is not None:
                try:
                    self.amcl_process.terminate()
                    self.amcl_process.wait(timeout=2)
                except Exception:
                    pass


def main():
    rclpy.init()
    node = StartLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
