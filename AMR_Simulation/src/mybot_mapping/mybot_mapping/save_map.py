#!/usr/bin/env python3
#!/usr/bin/env python3
"""map_save_proxy_node.py

Listens on `/mapping_tool/save_map` for JSON payloads and saves maps using
`map_saver_cli`. Publishes result JSON to `/mapping_tool/save_map_result`.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import json
from datetime import datetime

from mybot_common.constants import MAPS_DIR_STR
from mybot_common.topic_names import MAPPING_SAVE_MAP_TOPIC, MAPPING_SAVE_MAP_RESULT_TOPIC
from mybot_common.command_utils import extract_map_name, sanitize_map_name
from mybot_common.status_utils import format_status_json, create_status_payload

MAPS_DIR = MAPS_DIR_STR


class MapSaveProxy(Node):
    def __init__(self):
        super().__init__('map_save_proxy')
        # subscribe to mapping_tool topic used by the app
        self.sub = self.create_subscription(
            String, MAPPING_SAVE_MAP_TOPIC, self.on_cmd, 10)
        self.pub = self.create_publisher(
            String, MAPPING_SAVE_MAP_RESULT_TOPIC, 10)
        self.get_logger().info(f'Map save proxy ready, MAPS_DIR={MAPS_DIR}')

    def on_cmd(self, msg: String):
        # Extract map name robustly
        raw_map_name = extract_map_name(msg.data)
        safe_name = sanitize_map_name(raw_map_name)

        if not safe_name:
            # fallback to timestamped default
            safe_name = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        os.makedirs(MAPS_DIR, exist_ok=True)
        full_path = os.path.join(MAPS_DIR, safe_name)

        try:
            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", full_path, "--ros-args", "-p", "save_map_timeout:=5.0"],
                capture_output=True, text=True, timeout=30
            )
        except subprocess.TimeoutExpired:
            self.publish_result({"status": "error", "message": "Timeout when saving map"})
            return

        if result.returncode == 0:
            self.publish_result({
                "status": "ok",
                "map_name": safe_name,
                "saved_path": f"{full_path}.yaml"
            })
        else:
            self.publish_result({"status": "error", "message": result.stderr})

    def publish_result(self, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MapSaveProxy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
