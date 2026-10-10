#!/usr/bin/env python3
"""map_manager_node.py

Cầu nối giữa Android/Tablet và hệ thống file map trên máy chạy ROS.

Subscribe : /map_manager/get_available_maps_req  (std_msgs/Bool)
             → Nhận True từ Android để kích hoạt quét map.
Publish   : /map_manager/available_maps           (std_msgs/String)
             → Trả về JSON array tên map: '["restaurantA", "lab20"]'
"""

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mybot_common.constants import MAPS_DIR_STR
from mybot_common.topic_names import (
    MAP_MANAGER_GET_AVAILABLE_MAPS_REQ_TOPIC,
    MAP_MANAGER_AVAILABLE_MAPS_TOPIC,
)

MAPS_DIR = MAPS_DIR_STR


class MapManagerNode(Node):
    def __init__(self):
        super().__init__('map_manager')

        self._cached_maps_json = "[]"

        # Subscriber: chờ Android/Tablet gửi True
        self.sub = self.create_subscription(
            Bool,
            MAP_MANAGER_GET_AVAILABLE_MAPS_REQ_TOPIC,
            self.on_request,
            10
        )

        # Publisher: trả về danh sách map dạng JSON string
        self.pub = self.create_publisher(
            String,
            MAP_MANAGER_AVAILABLE_MAPS_TOPIC,
            10
        )

        # Quét và xuất bản danh sách map ngay khi khởi động
        self.publish_maps()

        # Timer định kỳ 1.0s đảm bảo App/Tablet vừa kết nối WebSocket là có danh sách map ngay
        self.timer = self.create_timer(1.0, self._periodic_publish)

        self.get_logger().info(
            f'MapManager ready — quét thư mục: {MAPS_DIR}'
        )

    def _periodic_publish(self):
        if self._cached_maps_json:
            out = String()
            out.data = self._cached_maps_json
            self.pub.publish(out)

    def on_request(self, msg: Bool):
        """Callback khi nhận request từ Android."""
        if not msg.data:
            return
        self.publish_maps()

    def publish_maps(self):
        map_names = self._scan_maps()
        self._cached_maps_json = json.dumps(map_names)
        out = String()
        out.data = self._cached_maps_json
        self.pub.publish(out)
        self.get_logger().info(
            f'📤 Published {len(map_names)} available maps: {map_names}'
        )

    def _scan_maps(self) -> list:
        """Quét MAPS_DIR, trả về list tên map (bỏ đuôi .yaml), đã sắp xếp."""
        try:
            entries = os.listdir(MAPS_DIR)
        except FileNotFoundError:
            self.get_logger().error(
                f'Không tìm thấy thư mục map: {MAPS_DIR}'
            )
            return []

        names = [
            os.path.splitext(f)[0]
            for f in sorted(entries)
            if f.endswith('.yaml')
        ]
        return names


def main():
    rclpy.init()
    node = MapManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
