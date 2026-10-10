#!/usr/bin/env python3
"""save_edited_map.py

Node tiếp nhận ảnh bản đồ đã chỉnh sửa (Base64) từ UI/App,
lưu ghi đè lên file map gốc (hoặc lưu map mới) và tự động yêu cầu Nav2 reload lại map.

Topic Subscribe: /mapping_tool/save_edited_map (std_msgs/String)
  Payload JSON:
  {
    "map_name": "map_lab20",
    "image_base64": "data:image/png;base64,iVBORw0...",
    "overwrite": true
  }

Topic Publish: /mapping_tool/save_edited_map_result (std_msgs/String)
  Output JSON:
  {
    "status": "ok" | "error",
    "map_name": "map_lab20",
    "message": "..."
  }
"""

import base64
import json
import os
import re
from pathlib import Path
from PIL import Image
import io

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.srv import LoadMap

from mybot_common.constants import MAPS_DIR
from mybot_common.topic_names import (
    MAPPING_SAVE_EDITED_MAP_TOPIC,
    MAPPING_SAVE_EDITED_MAP_RESULT_TOPIC,
    NAV2_MAP_SERVER_LOAD_MAP_SERVICE
)
from mybot_common.command_utils import sanitize_map_name
from mybot_common.status_utils import format_status_json, create_status_payload


class SaveEditedMapNode(Node):
    def __init__(self):
        super().__init__('save_edited_map_proxy')

        self.subscription = self.create_subscription(
            String,
            MAPPING_SAVE_EDITED_MAP_TOPIC,
            self.on_save_edited_cmd,
            10
        )

        self.publisher = self.create_publisher(
            String,
            MAPPING_SAVE_EDITED_MAP_RESULT_TOPIC,
            10
        )

        self.nav2_client = self.create_client(LoadMap, NAV2_MAP_SERVER_LOAD_MAP_SERVICE)

        self.get_logger().info(f'SaveEditedMapNode ready. MAPS_DIR={MAPS_DIR}')

    def on_save_edited_cmd(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict) and 'data' in payload and isinstance(payload['data'], dict):
                payload = payload['data']
        except Exception as e:
            self.get_logger().error(f'Invalid JSON payload: {e}')
            self.publish_result({'status': 'error', 'message': f'Invalid JSON: {e}'})
            return

        map_name = payload.get('map_name', '').strip()
        image_b64 = payload.get('image_base64', '').strip()
        overwrite = payload.get('overwrite', True)

        if not map_name:
            self.publish_result({'status': 'error', 'message': 'Missing map_name'})
            return

        if not image_b64:
            self.publish_result({'status': 'error', 'message': 'Missing image_base64'})
            return

        # Sanitize map_name
        safe_name = re.sub(r'[^a-zA-Z0-9_\-]', '', map_name)
        if not safe_name:
            self.publish_result({'status': 'error', 'message': 'Invalid map_name format'})
            return

        # Tách phần header "data:image/png;base64," nếu có
        if ',' in image_b64:
            image_b64 = image_b64.split(',', 1)[1]

        try:
            image_bytes = base64.b64decode(image_b64)
            img = Image.open(io.BytesIO(image_bytes))
        except Exception as e:
            self.get_logger().error(f'Failed to decode Base64 image: {e}')
            self.publish_result({'status': 'error', 'message': f'Base64 decode error: {e}'})
            return

        MAPS_DIR.mkdir(parents=True, exist_ok=True)

        target_pgm = MAPS_DIR / f'{safe_name}.pgm'
        target_png = MAPS_DIR / f'{safe_name}.png'
        target_yaml = MAPS_DIR / f'{safe_name}.yaml'

        # Tìm xem file map gốc dùng file ảnh định dạng nào (.pgm hay .png)
        if target_pgm.exists():
            save_img_path = target_pgm
        elif target_png.exists():
            save_img_path = target_png
        else:
            save_img_path = target_pgm

        try:
            # Lưu ảnh đã sửa ghi đè lên file map gốc
            # Chuyển đổi sang mode L (grayscale) phù hợp với ROS OccupancyGrid
            img_gray = img.convert('L')
            img_gray.save(save_img_path)
            self.get_logger().info(f'Saved edited map image to {save_img_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save image file: {e}')
            self.publish_result({'status': 'error', 'message': f'Image save error: {e}'})
            return

        # Kiểm tra xem có file YAML không, nếu không tạo YAML mặc định
        if not target_yaml.exists():
            yaml_content = f"""image: {save_img_path.name}
mode: trinary
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
            with open(target_yaml, 'w') as f:
                f.write(yaml_content)
            self.get_logger().info(f'Created default YAML at {target_yaml}')

        # Reload map vào Nav2 map_server
        self.reload_nav2_map(safe_name, target_yaml)

    def reload_nav2_map(self, map_name: str, yaml_path: Path):
        if not self.nav2_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('Nav2 map_server service not available for auto-reload.')
            self.publish_result({
                'status': 'ok',
                'map_name': map_name,
                'message': 'Map saved successfully, but Nav2 map_server was not online to auto-reload.'
            })
            return

        req = LoadMap.Request()
        req.map_url = str(yaml_path)
        future = self.nav2_client.call_async(req)
        future.add_done_callback(
            lambda fut: self.on_reload_complete(fut, map_name)
        )

    def on_reload_complete(self, future, map_name: str):
        try:
            res = future.result()
            if res.result == LoadMap.Response.RESULT_SUCCESS:
                self.get_logger().info(f'Successfully reloaded map "{map_name}" into Nav2!')
                self.publish_result({
                    'status': 'ok',
                    'map_name': map_name,
                    'message': f'Map "{map_name}" edited, saved, and reloaded successfully.'
                })
            else:
                self.get_logger().error(f'Nav2 LoadMap failed with code: {res.result}')
                self.publish_result({
                    'status': 'warning',
                    'map_name': map_name,
                    'message': f'Map saved, but Nav2 reload failed with code {res.result}'
                })
        except Exception as e:
            self.get_logger().error(f'Error calling Nav2 LoadMap service: {e}')
            self.publish_result({
                'status': 'warning',
                'map_name': map_name,
                'message': f'Map saved, but reload error: {e}'
            })

    def publish_result(self, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SaveEditedMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
