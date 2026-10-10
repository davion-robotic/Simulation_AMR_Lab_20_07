#!/usr/bin/env python3
"""
viz_navgraph.py (mybot_rmf)
----------------------------
Hiển thị Đồ thị Topology theo CHUẨN ĐỒ HỌA MẶC ĐỊNH CỦA OPEN-RMF:
- Làn đường dải màu cam bản to (Orange Wide Band Lanes - width 0.35m, alpha 0.6)
- Trạm đỗ hình khối vuông màu cam (Orange Cubes)
- Trạm sạc (charger/start) hình trụ màu vàng (Yellow Cylinder)
- Tên trạm nổi phía trên chữ màu cam/trắng
- Chuẩn hóa QoS (TRANSIENT_LOCAL) tương thích 100% với RViz2 RMF Display.
"""

import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory


import json
from std_msgs.msg import String
from mybot_common.command_utils import parse_payload_dict, sanitize_map_name
from mybot_common.topic_names import TOPOLOGY_RELOAD_TOPIC, MAP_MANAGER_LOAD_MAP_TOPIC

class NavGraphViz(Node):
    def __init__(self):
        super().__init__('navgraph_viz')

        self.active_map_name = "map_lab20"

        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.pub_map = self.create_publisher(MarkerArray, '/map_markers', qos)
        self.pub_nav = self.create_publisher(MarkerArray, '/navgraph_markers', qos)

        self.sub_reload = self.create_subscription(
            String, TOPOLOGY_RELOAD_TOPIC, self.on_reload, 10)
        self.sub_load_map = self.create_subscription(
            String, MAP_MANAGER_LOAD_MAP_TOPIC, self.on_map_loaded, 10)

        self.data = {}
        self.reload_graph()

        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info("Official Open-RMF Standard Visualizer initialized with TRANSIENT_LOCAL QoS & Auto-Reload.")

    def get_graph_path(self, map_name: str = None) -> str:
        target = sanitize_map_name(map_name) or self.active_map_name
        candidates = [
            f'/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_rmf/maps/{target}_topology.yaml',
            f'/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_rmf/maps/{target}topology.yaml',
            f'/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_rmf/maps/map_lab20_topology.yaml'
        ]
        try:
            _pkg_share = get_package_share_directory('mybot_rmf')
            candidates.insert(0, os.path.join(_pkg_share, 'maps', f'{target}_topology.yaml'))
        except Exception:
            pass

        for p in candidates:
            if os.path.exists(p):
                return p
        return candidates[0]

    def on_map_loaded(self, msg: String):
        payload = parse_payload_dict(msg.data)
        map_name = sanitize_map_name(payload.get("map_name") or payload.get("data"))
        if map_name:
            self.active_map_name = map_name
            self.get_logger().info(f"Map switched to '{map_name}', reloading visualizer...")
            self.reload_graph()
            self.publish()

    def on_reload(self, msg: String):
        payload = parse_payload_dict(msg.data)
        map_name = sanitize_map_name(payload.get("map_name"))
        if map_name:
            self.active_map_name = map_name
        self.get_logger().info("Received Topology Reload signal, re-reading graph file...")
        self.reload_graph()
        self.publish()

    def reload_graph(self):
        graph_path = self.get_graph_path(self.active_map_name)
        try:
            if os.path.exists(graph_path):
                with open(graph_path, 'r', encoding='utf-8') as f:
                    self.data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Error reloading graph file '{graph_path}': {e}")

    def publish(self):
        markers = MarkerArray()

        # Xóa toàn bộ marker cũ trên RViz2 trước khi vẽ lại
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        markers.markers.append(del_marker)

        levels = self.data.get('levels', {})
        if not levels:
            self.pub_map.publish(markers)
            self.pub_nav.publish(markers)
            return

        level_name = next(iter(levels))
        verts = levels[level_name].get('vertices', [])
        lanes = levels[level_name].get('lanes', [])
        now = self.get_clock().now().to_msg()

        # 1. Hiển thị các Làn đường Topology (Lanes) - Dải màu cam bản to chuẩn Open-RMF
        for i, lane in enumerate(lanes):
            l = Marker()
            l.header.frame_id = 'map'
            l.header.stamp = now
            l.ns = 'rmf_lanes'
            l.id = i
            l.type = Marker.LINE_STRIP
            l.action = Marker.ADD

            idx1, idx2 = lane[0], lane[1]
            if idx1 < len(verts) and idx2 < len(verts):
                a, b = verts[idx1], verts[idx2]
                l.points = [
                    Point(x=float(a[0]), y=float(a[1]), z=0.03),
                    Point(x=float(b[0]), y=float(b[1]), z=0.03)
                ]

            # Bề rộng dải làn màu cam 0.35m chuẩn Open-RMF
            l.scale.x = 0.35

            # Màu cam trong suốt chuẩn Open-RMF (RGB: 1.0, 0.45, 0.0, Alpha: 0.65)
            l.color.r = 1.0
            l.color.g = 0.45
            l.color.b = 0.0
            l.color.a = 0.65
            markers.markers.append(l)

        # 2. Hiển thị các Nút Trạm Topology (Waypoints / Stations)
        for i, v in enumerate(verts):
            opts = v[2] if len(v) > 2 else {}
            name = opts.get('name', str(i))
            is_charger = opts.get('is_charger', False) or name == 'start' or 'charger' in name

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'rmf_waypoints'
            m.id = i
            m.action = Marker.ADD
            m.pose.position.x = float(v[0])
            m.pose.position.y = float(v[1])
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0

            if is_charger:
                # Trạm sạc: Hình trụ màu vàng dầy chuẩn Open-RMF
                m.type = Marker.CYLINDER
                m.scale.x = m.scale.y = 0.45
                m.scale.z = 0.15
                m.color.r = 1.0
                m.color.g = 0.85
                m.color.b = 0.0
                m.color.a = 0.95
            else:
                # Trạm đỗ thường: Khối vuông màu cam chuẩn Open-RMF
                m.type = Marker.CUBE
                m.scale.x = m.scale.y = 0.38
                m.scale.z = 0.1
                m.color.r = 1.0
                m.color.g = 0.45
                m.color.b = 0.0
                m.color.a = 0.9

            markers.markers.append(m)

            # 3. Tên trạm đỗ (Text Labels)
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp = now
            t.ns = 'rmf_labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(v[0])
            t.pose.position.y = float(v[1]) + 0.35
            t.pose.position.z = 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color.r = 1.0
            t.color.g = 0.55
            t.color.b = 0.1
            t.color.a = 1.0
            t.text = name
            markers.markers.append(t)

        self.pub_map.publish(markers)
        self.pub_nav.publish(markers)


def main():
    rclpy.init()
    node = NavGraphViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
