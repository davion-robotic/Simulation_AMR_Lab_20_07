#!/usr/bin/env python3
"""load_map.py — Cầu nối load map từ Android/Tablet sang Nav2 / Standalone Map Server.

Android gửi:
  topic : /map_manager/load_map  (std_msgs/String)
  msg   : data = "map_lab20"   (plain string hoặc JSON {"data": "map_lab20"})

Nếu Nav2 map_server service sẵn sàng → Gọi service /map_server/load_map để đổi map.
Nếu Nav2 chưa chạy → Tự động khởi chạy Standalone nav2_map_server & kích hoạt lifecycle để hiển thị map trên /map ngay lập tức!

Kết quả trả về:
  topic : /map_manager/load_map_result  (std_msgs/String)
  msg   : JSON {"status": "ok", "map_name": "..."}  hoặc  {"status": "error"}
"""

import json
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from nav2_msgs.srv import LoadMap
from rclpy.node import Node
from std_msgs.msg import String

from mybot_common.constants import MAPS_DIR
from mybot_common.topic_names import (
    MAP_MANAGER_LOAD_MAP_TOPIC,
    MAP_MANAGER_LOAD_MAP_RESULT_TOPIC,
    NAV2_MAP_SERVER_LOAD_MAP_SERVICE,
    GOAL_AVAILABLE_GOALS_TOPIC,
    GOAL_LIST_TOPIC,
)
from mybot_common.command_utils import extract_map_name, sanitize_map_name
from mybot_common.status_utils import format_status_json, create_status_payload


class MapLoadProxy(Node):
    def __init__(self):
        super().__init__('map_load_proxy')
        if not self.has_parameter('use_sim_time'):
            try:
                self.declare_parameter('use_sim_time', True)
            except Exception:
                pass
        self.client = self.create_client(LoadMap, NAV2_MAP_SERVER_LOAD_MAP_SERVICE)
        self.standalone_proc = None
        self.current_loaded_map = None

        # Android publish tên map dạng plain string hoặc JSON vào topic này
        self.subscription = self.create_subscription(
            String, MAP_MANAGER_LOAD_MAP_TOPIC, self.on_command, 10)

        # Kết quả trả về cho Android
        self.publisher = self.create_publisher(
            String, MAP_MANAGER_LOAD_MAP_RESULT_TOPIC, 10)

        # Publishers tự động đồng bộ Goal khi load map
        self.pub_available_goals = self.create_publisher(String, GOAL_AVAILABLE_GOALS_TOPIC, 10)
        self.pub_goals_list = self.create_publisher(String, GOAL_LIST_TOPIC, 10)

        self.get_logger().info('Map load proxy ready, MAPS_DIR=%s' % MAPS_DIR)

    def on_command(self, message: String):
        raw_name = extract_map_name(message.data)
        map_name = sanitize_map_name(raw_name)
        if map_name:
            self.load_map_by_name(map_name)

    def load_map_by_name(self, map_name: str):
        if not map_name:
            self.get_logger().warn('Tên map không hợp lệ')
            self.publish_result({'status': 'error', 'message': 'map_name không hợp lệ'})
            return

        map_yaml = MAPS_DIR / f'{map_name}.yaml'
        if not map_yaml.is_file():
            if map_name.endswith('.yaml'):
                map_yaml = MAPS_DIR / map_name
                map_name = map_name[:-5]

        if not map_yaml.is_file():
            self.get_logger().error(f'Không tìm thấy file map: {map_yaml}')
            self.publish_result({
                'status': 'error',
                'message': f'Không tìm thấy file map: {map_yaml}',
            })
            return

        self.current_loaded_map = map_name
        self.get_logger().info(f'Processing load_map request for "{map_name}" at path: {map_yaml}')

        # Đồng bộ danh sách goal tự động khi load map thành công
        self._publish_map_goals(map_name)

        # If Nav2 service is running, call service
        if self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Calling Nav2 load_map service for {map_name}...')
            request = LoadMap.Request()
            request.map_url = str(map_yaml)
            future = self.client.call_async(request)
            future.add_done_callback(
                lambda completed, name=map_name, path=map_yaml:
                self.on_load_complete(completed, name, path))
            return

        # Otherwise launch standalone map_server and activate its lifecycle
        self.get_logger().info(f'Nav2 service not running. Launching Standalone map_server for {map_yaml}...')
        try:
            subprocess.run(['pkill', '-f', 'nav2_map_server'], capture_output=True, text=True)
            time.sleep(0.2)

            use_sim_val = self.get_parameter('use_sim_time').value if self.has_parameter('use_sim_time') else True
            use_sim_str = "true" if use_sim_val else "false"

            self.standalone_proc = subprocess.Popen([
                'ros2', 'run', 'nav2_map_server', 'map_server',
                '--ros-args',
                '-p', f'yaml_filename:={map_yaml}',
                '-p', f'use_sim_time:={use_sim_str}'
            ])

            threading.Thread(target=self._activate_standalone_map_server, daemon=True).start()

            self.get_logger().info(f'Standalone map_server launched successfully for map {map_name}')
            self.publish_result({
                'status': 'ok',
                'map_name': map_name,
                'mode': 'standalone',
                'loaded_path': str(map_yaml),
            })
        except Exception as error:
            self.get_logger().error(f'Failed to launch standalone map_server: {error}')
            self.publish_result({'status': 'error', 'message': str(error)})

    def _publish_map_goals(self, map_name: str):
        try:
            from mybot_goal.goal_storage import GoalStorage
            storage = GoalStorage()
            goals = storage.load_goals(map_name)
            ui_goals = []
            for g in goals:
                pose = g.get("pose", {}) if isinstance(g.get("pose"), dict) else {}
                ui_goals.append({
                    "goal_id": str(g.get("goal_id") or g.get("name") or "").strip(),
                    "name": str(g.get("name") or g.get("goal_id") or "").strip(),
                    "map_name": map_name,
                    "x": float(pose.get("x", 0.0)),
                    "y": float(pose.get("y", 0.0)),
                    "theta": float(pose.get("theta", 0.0))
                })
            out_available = String()
            out_available.data = json.dumps(ui_goals)
            self.pub_available_goals.publish(out_available)

            out_list = String()
            out_list.data = json.dumps({"status": "ok", "map_name": map_name, "goals": goals})
            self.pub_goals_list.publish(out_list)
        except Exception as e:
            self.get_logger().warn(f"Could not publish map goals for {map_name}: {e}")

    def _activate_standalone_map_server(self):
        """Safely configure and activate standalone map_server lifecycle by checking current state."""
        time.sleep(1.0)
        try:
            res = subprocess.run([
                'ros2', 'service', 'call', '/map_server/get_state',
                'lifecycle_msgs/srv/GetState', '{}'
            ], capture_output=True, text=True)
            out = res.stdout.lower()

            # If already active, nothing to do
            if 'label: active' in out or "label='active'" in out or 'label="active"' in out or 'id=3' in out or 'id: 3' in out:
                self.get_logger().info('Standalone map_server is already active.')
                return

            # If unconfigured -> transition 1 (CONFIGURE) to become inactive
            if 'unconfigured' in out or 'id: 1' in out or 'id=1' in out:
                self.get_logger().info('Configuring standalone map_server (transition 1)...')
                subprocess.run([
                    'ros2', 'service', 'call', '/map_server/change_state',
                    'lifecycle_msgs/srv/ChangeState', '{"transition": {"id": 1}}'
                ], capture_output=True, text=True)
                time.sleep(0.5)

            # Re-check state before calling ACTIVATE
            res2 = subprocess.run([
                'ros2', 'service', 'call', '/map_server/get_state',
                'lifecycle_msgs/srv/GetState', '{}'
            ], capture_output=True, text=True)
            out2 = res2.stdout.lower()

            if 'label: active' in out2 or "label='active'" in out2 or 'label="active"' in out2 or 'id=3' in out2 or 'id: 3' in out2:
                self.get_logger().info('Standalone map_server is already active.')
                return

            # If inactive -> transition 3 (ACTIVATE)
            if 'inactive' in out2 or 'id: 2' in out2 or 'id=2' in out2:
                self.get_logger().info('Activating standalone map_server (transition 3)...')
                subprocess.run([
                    'ros2', 'service', 'call', '/map_server/change_state',
                    'lifecycle_msgs/srv/ChangeState', '{"transition": {"id": 3}}'
                ], capture_output=True, text=True)

            self.get_logger().info('Standalone map_server activated successfully! Map is live on /map.')
        except Exception as e:
            self.get_logger().error(f'Failed to activate standalone map_server lifecycle: {e}')

    def on_load_complete(self, future, map_name: str, map_yaml: Path):
        try:
            response = future.result()
            if response.result == LoadMap.Response.RESULT_SUCCESS:
                self.publish_result({
                    'status': 'ok',
                    'map_name': map_name,
                    'loaded_path': str(map_yaml),
                })
            else:
                self.publish_result({
                    'status': 'error',
                    'message': f'Nav2 không thể nạp map (mã {response.result})',
                })
        except Exception as error:
            self.publish_result({'status': 'error', 'message': str(error)})

    def publish_result(self, data: dict):
        message = String()
        message.data = json.dumps(data)
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = MapLoadProxy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
