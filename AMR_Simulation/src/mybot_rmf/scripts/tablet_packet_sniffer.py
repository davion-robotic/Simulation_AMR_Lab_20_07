#!/usr/bin/env python3
"""
tablet_packet_sniffer.py - Bắt toàn bộ dữ liệu & gói tin Tablet gửi xuống máy tính điều khiển Robot.

Theo dõi tất cả các kênh:
1. Topology (Thêm / Sửa / Xóa Vertex & Lane, Lưu đồ thị)
2. Goal / Waypoint (Tạo, Sửa, Xóa Goal, Lệnh di chuyển Go To Goal)
3. Map Management (Tải bản đồ, Quét SLAM, Lưu bản đồ mới)
4. Localization (AMCL Start/Stop, Initial Pose, Reset Pose)
5. Navigation & Điều khiển vận tốc (Goal Pose, Cmd Vel)
6. Giám sát kết nối WebSocket (Port 9090)

Sử dụng:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/mybot_rmf/scripts/tablet_packet_sniffer.py
"""

import json
import os
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

# Màu terminal ANSI
RESET   = '\033[0m'
BOLD    = '\033[1m'
GREEN   = '\033[92m'
YELLOW  = '\033[93m'
RED     = '\033[91m'
CYAN    = '\033[96m'
BLUE    = '\033[94m'
MAGENTA = '\033[95m'
GRAY    = '\033[90m'
BG_BLUE = '\033[44m'

# =========================================================================
# DANH SÁCH TẤT CẢ CÁC TOPIC TABLET/APP CÓ THỂ GỬI LỆNH VÀO
# =========================================================================
WATCHED_STRING_TOPICS = [
    # --- TOPOLOGY (Vertex & Lane) ---
    '/topology_manager/save_topology',
    '/map_manager/save_topology',
    '/topology_manager/edit_topology',
    '/map_manager/edit_topology',
    '/topology_manager/delete_vertex',
    '/topology_manager/remove_vertex',
    '/map_manager/delete_vertex',
    '/map_manager/remove_vertex',
    '/topology_manager/add_vertex',
    '/topology_manager/create_vertex',
    '/map_manager/add_vertex',
    '/map_manager/create_vertex',
    '/topology_manager/edit_vertex',
    '/topology_manager/update_vertex',
    '/map_manager/edit_vertex',
    '/map_manager/update_vertex',
    '/topology_manager/delete_lane',
    '/topology_manager/remove_lane',
    '/map_manager/delete_lane',
    '/map_manager/remove_lane',
    '/topology_manager/add_lane',
    '/topology_manager/create_lane',
    '/map_manager/add_lane',
    '/map_manager/create_lane',
    '/topology_manager/edit_lane',
    '/topology_manager/update_lane',
    '/map_manager/edit_lane',
    '/map_manager/update_lane',
    '/topology_manager/result',

    # --- GOAL / WAYPOINT MANAGEMENT ---
    '/goal_manager/create_goal',
    '/goal_manager/update_goal',
    '/goal_manager/delete_goal',
    '/goal_manager/get_goals_req',
    '/goal_manager/go_to_goal',
    '/goal_manager/cancel_goal',
    '/goal_manager/result',

    # --- MAP MANAGEMENT & SLAM ---
    '/map_manager/load_map',
    '/map_manager/load_map_result',
    '/map_manager/get_available_maps_req',
    '/mapping_tool/scan',
    '/mapping_tool/save_map',
    '/mapping_tool/save_edited_map',

    # --- LOCALIZATION ---
    '/localization/start',
    '/localization/stop',
    '/localization/initial_pose',
    '/localization/reset_pose',
    '/initialpose',
]


def get_topic_color(topic: str) -> str:
    t = topic.lower()
    if 'delete' in t or 'remove' in t or 'cancel' in t or 'stop' in t:
        return RED
    if 'add' in t or 'create' in t or 'start' in t or 'scan' in t:
        return GREEN
    if 'edit' in t or 'update' in t or 'save' in t or 'load' in t or 'initial' in t:
        return YELLOW
    if 'result' in t or 'response' in t:
        return CYAN
    if 'go_to' in t or 'goal' in t:
        return MAGENTA
    return BLUE


def format_json_payload(data_str: str) -> str:
    """Định dạng và làm nổi bật các trường quan trọng."""
    try:
        data = json.loads(data_str)
        if not isinstance(data, dict):
            return f"  {BOLD}DATA:{RESET} {repr(data_str)}"

        lines = []

        # 1. Action / Command
        action = None
        for k in ["action", "command", "cmd", "op", "type", "operation", "method"]:
            if k in data and data[k]:
                action = data[k]
                break
        if action:
            lines.append(f"  {BOLD}🎯 ACTION  :{RESET} {CYAN}{BOLD}{action}{RESET}")

        # 2. Map Name
        map_name = data.get("map_name") or data.get("mapName") or data.get("map")
        if map_name:
            lines.append(f"  {BOLD}🗺️ MAP     :{RESET} {YELLOW}{map_name}{RESET}")

        # 3. Vertex / Point Name
        v_name = data.get("name") or data.get("vertex_name") or data.get("vertexName") or data.get("goal_id") or data.get("goalId") or data.get("id")
        if v_name:
            lines.append(f"  {BOLD}📍 VERTEX  :{RESET} {GREEN}{v_name}{RESET}")

        # 4. Lane / Route
        from_v = data.get("from") or data.get("fromVertex") or data.get("start") or data.get("start_vertex") or data.get("u")
        to_v = data.get("to") or data.get("toVertex") or data.get("end") or data.get("end_vertex") or data.get("v")
        if from_v is not None or to_v is not None:
            bidi = data.get("is_bidirectional", data.get("isBidirectional", data.get("bidirectional", True)))
            arrow = "<=====>" if bidi else "=====>"
            lines.append(f"  {BOLD}🛣️ LANE    :{RESET} {MAGENTA}'{from_v}' {arrow} '{to_v}'{RESET}")

        # 5. Position (x, y, yaw)
        if "x" in data or "y" in data:
            x = data.get("x", 0.0)
            y = data.get("y", 0.0)
            yaw = data.get("yaw", data.get("theta", 0.0))
            lines.append(f"  {BOLD}📐 POSE    :{RESET} x={x}, y={y}, yaw={yaw}")

        # 6. Status & Message
        if "status" in data:
            st = data["status"]
            st_color = GREEN if st == "ok" else RED
            lines.append(f"  {BOLD}📊 STATUS  :{RESET} {st_color}{BOLD}{st.upper()}{RESET}")
        if "message" in data:
            lines.append(f"  {BOLD}💬 MESSAGE :{RESET} {data['message']}")

        # Full Pretty JSON
        pretty_json = json.dumps(data, indent=4, ensure_ascii=False)
        indented = "\n".join("    " + l for l in pretty_json.splitlines())
        lines.append(f"\n  {GRAY}--- FULL JSON PAYLOAD ---{RESET}\n{indented}")

        return "\n".join(lines)
    except json.JSONDecodeError:
        return f"  {BOLD}RAW TEXT:{RESET} {data_str}"


class TabletPacketSnifferNode(Node):
    def __init__(self):
        super().__init__('tablet_packet_sniffer')
        self.msg_count = 0
        self.lock = threading.Lock()

        # In banner
        print(f"\n{BOLD}{BG_BLUE}{' ' * 70}{RESET}")
        print(f"{BOLD}{BG_BLUE}  📱 TABLET PACKET SNIFFER - GIÁM SÁT CHI TIẾT DỮ LIỆU TỪ TABLET  {RESET}")
        print(f"{BOLD}{BG_BLUE}{' ' * 70}{RESET}\n")
        print(f"  • {GREEN}Đang lắng nghe {len(WATCHED_STRING_TOPICS)} topics ROS 2 từ Tablet / Web UI...{RESET}")
        print(f"  • {CYAN}WebSocket Port: 9090 (rosbridge_server){RESET}")
        print(f"  • {GRAY}Nhấn Ctrl + C để dừng theo dõi.{RESET}\n")

        self.check_websocket_connections()

        # Đăng ký lắng nghe toàn bộ các String topics
        self._subs = []
        for topic in WATCHED_STRING_TOPICS:
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, t=topic: self._on_string_msg(msg, t),
                10
            )
            self._subs.append(sub)

        # Lắng nghe PoseStamped (Goal Pose)
        self.sub_goal_pose = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._on_goal_pose_msg,
            10
        )

        print(f"{BOLD}{'─' * 70}{RESET}")
        print(f"{BOLD}  🚀 SẴN SÀNG! ĐANG CHỜ GÓI TIN TỪ TABLET...{RESET}")
        print(f"{BOLD}{'─' * 70}{RESET}\n")

    def check_websocket_connections(self):
        """Kiểm tra có Tablet nào đang kết nối cổng 9090 không."""
        try:
            out = subprocess.check_output("ss -tnp | grep 9090 || true", shell=True, text=True).strip()
            if out:
                lines = out.splitlines()
                established = [l for l in lines if "ESTAB" in l]
                if established:
                    print(f"  {GREEN}✅ Đã phát hiện {len(established)} kết nối WebSocket từ Tablet tới cổng 9090!{RESET}\n")
                else:
                    print(f"  {YELLOW}⚠️ Cổng 9090 đang mở, nhưng chưa có Tablet nào kết nối WebSocket.{RESET}\n")
            else:
                print(f"  {RED}❌ Cổng 9090 chưa mở. Hãy chắc chắn đã chạy 'ros2 launch mybot_navigation2_tablet tablet_bridge.launch.py'{RESET}\n")
        except Exception:
            pass

    def _on_string_msg(self, msg: String, topic: str):
        with self.lock:
            self.msg_count += 1
            now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            color = get_topic_color(topic)

            print(f"\n{color}{BOLD}{'=' * 70}{RESET}")
            print(f"{color}{BOLD}  📦 GÓI TIN #{self.msg_count:04d}  |  THỜI GIAN: {now}{RESET}")
            print(f"{color}{BOLD}  📡 TOPIC: {topic}{RESET}")
            print(f"{color}{'─' * 70}{RESET}")
            print(format_json_payload(msg.data))
            print(f"{color}{'=' * 70}{RESET}\n")

    def _on_goal_pose_msg(self, msg: PoseStamped):
        with self.lock:
            self.msg_count += 1
            now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            p = msg.pose.position
            o = msg.pose.orientation
            print(f"\n{MAGENTA}{BOLD}{'=' * 70}{RESET}")
            print(f"{MAGENTA}{BOLD}  🎯 GÓI TIN #{self.msg_count:04d}  |  THỜI GIAN: {now}{RESET}")
            print(f"{MAGENTA}{BOLD}  📡 TOPIC: /goal_pose (Nav2 Goal Coordinate){RESET}")
            print(f"{MAGENTA}{'─' * 70}{RESET}")
            print(f"  {BOLD}Frame ID :{RESET} {msg.header.frame_id}")
            print(f"  {BOLD}Position :{RESET} x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
            print(f"  {BOLD}Orient   :{RESET} z={o.z:.3f}, w={o.w:.3f}")
            print(f"{MAGENTA}{'=' * 70}{RESET}\n")


def main():
    rclpy.init()
    node = TabletPacketSnifferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Đã dừng giám sát. Tổng số gói tin bắt được: {node.msg_count}{RESET}\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
