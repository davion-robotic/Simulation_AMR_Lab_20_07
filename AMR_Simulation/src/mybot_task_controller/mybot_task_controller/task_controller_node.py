#!/usr/bin/env python3
"""
task_controller_node.py
-----------------------
Quản lý tác vụ điều khiển hành trình Robot AMR:
  1. Hỗ trợ điểm đơn lẻ (Single Goal via /goal_pose)
  2. Hỗ trợ tuần tra danh sách điểm (Multi-waypoint Patrol / Route)
  3. Quản lý thời gian dừng chờ tại từng điểm (Dwell Time)
  4. Quản lý hạn mức thời gian toàn bộ chu kỳ (Global Timeout, e.g. 2 giờ)
  5. Cơ chế PAUSE / RESUME thông minh:
     - Tạm dừng khi đang chạy (TRANSIT): Hủy Nav2 goal, giữ nguyên đích đến, dập phanh.
     - Tạm dừng khi đang chờ (DWELLING): Đóng băng bộ đếm countdown, giữ phanh.
     - Tiếp tục (RESUME): Đi tiếp hoặc đếm nốt số giây còn lại.
  6. Xử lý lỗi vật cản / kẹt đường (Retry & Auto Skip).
  7. Theo dõi pin: Tự động ngắt tour và chuyển hướng về trạm sạc khi pin < 20%.
  8. Xuất báo cáo trạng thái định kỳ 1.0 Hz qua topic /task_controller/status.
"""

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

try:
    from mybot_goal.goal_storage import GoalStorage
except ImportError:
    GoalStorage = None


class TaskControllerNode(Node):
    """Bộ điều khiển tác vụ và tuần tra Waypoint cho Robot AMR."""

    def __init__(self):
        super().__init__('task_controller_node')

        # ----------------- Khai báo Parameters an toàn -----------------
        self._declare_param_safe('cmd_topic', '/task_controller/command')
        self._declare_param_safe('status_topic', '/task_controller/status')
        self._declare_param_safe('cmd_vel_topic', '/cmd_vel')
        self._declare_param_safe('nav2_action_name', 'navigate_to_pose')
        self._declare_param_safe('default_map', 'map_lab20')
        self._declare_param_safe('battery_low_threshold', 20.0)
        self._declare_param_safe('max_retries_per_wp', 2)
        self._declare_param_safe('use_sim_time', True)

        cmd_topic = self.get_parameter('cmd_topic').value
        status_topic = self.get_parameter('status_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        nav2_action = self.get_parameter('nav2_action_name').value
        self.default_map = self.get_parameter('default_map').value
        self.battery_low_threshold = float(self.get_parameter('battery_low_threshold').value)
        self.max_retries_per_wp = int(self.get_parameter('max_retries_per_wp').value)

        # ----------------- Khởi tạo GoalStorage -----------------
        if GoalStorage is not None:
            self.storage = GoalStorage()
        else:
            self.storage = None
            self.get_logger().warn("⚠️ mybot_goal.goal_storage could not be imported. Map lookup will be limited.")

        # ----------------- Các biến trạng thái FSM -----------------
        # States: 'IDLE', 'NAVIGATING', 'WAITING', 'PAUSED', 'CANCELLED', 'RETURNING_HOME'
        self.state = 'IDLE'
        # Sub-states: 'NONE', 'TRANSIT', 'DWELLING'
        self.sub_state = 'NONE'
        self.paused_prev_state = 'NONE'
        self.is_paused = False

        # Mission types: 'NONE', 'SINGLE', 'MISSION', 'DELIVERY'
        self.mission_type = 'NONE'
        self.mission_id = ""
        self.current_map_name = self.default_map

        # Danh sách Waypoint tuần tự từng điểm
        self.patrol_waypoints = []
        self.current_wp_idx = 0
        self.loop_patrol = False
        self.on_finish_action = "HOME"  # Luôn luôn về HOME sau khi xong điểm cuối cùng!

        # Thống kê nhiệm vụ (Mission Statistics)
        self.completed_waypoints_count = 0
        self.skipped_waypoints_count = 0
        self.mission_start_epoch = 0.0

        # Bộ đếm thời gian
        self.dwell_time_remaining = 0.0

        # Trạng thái phần cứng & định vị
        self.battery_percent = 100.0
        self.current_goal_handle = None
        self.saved_goal_pose = None
        self.current_goal_id = ""
        self.current_goal_name = ""
        self.status_message = "Ready"

        # ----------------- Subscribers & Publishers -----------------
        self.cmd_sub = self.create_subscription(
            String, cmd_topic, self.command_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.single_goal_callback, 10
        )
        self.sub_robot_status = self.create_subscription(
            String, '/robot_status_simple', self.robot_status_cb, 10
        )
        self.sub_battery = self.create_subscription(
            BatteryState, '/battery_state', self.battery_state_cb, 10
        )

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Nav2 Action Client
        self.nav2_client = ActionClient(self, NavigateToPose, nav2_action)

        # Timer định kỳ 1.0 Hz (Cập nhật countdown và phát JSON lên UI)
        self.timer_1hz = self.create_timer(1.0, self.periodic_timer_1hz)

        self.get_logger().info('===============================================================')
        self.get_logger().info('🚀 Advanced Task Controller & Waypoint Patrol Node initialized!')
        self.get_logger().info(f'📡 Command Topic:       {cmd_topic}')
        self.get_logger().info(f'📊 Status Topic:        {status_topic}')
        self.get_logger().info(f'🔋 Low Battery Limit:   {self.battery_low_threshold}%')
        self.get_logger().info(f'🔄 Max Retry per Point: {self.max_retries_per_wp}')
        self.get_logger().info('===============================================================')

    def _declare_param_safe(self, name: str, default_val):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_val)

    # =========================================================================
    # LẮNG NGHE PIN & TRẠNG THÁI HỆ THỐNG
    # =========================================================================
    def robot_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if "battery_percent" in data and data["battery_percent"] is not None:
                self.battery_percent = float(data["battery_percent"])
        except Exception:
            pass

    def battery_state_cb(self, msg: BatteryState):
        if msg.percentage is not None and msg.percentage > 0.0:
            # message BatteryState có thể là 0.0-1.0 hoặc 0-100%
            self.battery_percent = msg.percentage if msg.percentage > 1.0 else (msg.percentage * 100.0)

    # =========================================================================
    # BỘ ĐẾM CHU KỲ 1 HZ (TIMERS, COUNTDOWN, SAFETY)
    # =========================================================================
    def periodic_timer_1hz(self):
        # 1. Đếm ngược thời gian chờ tại điểm (Dwell Time)
        if self.state in ['WAITING', 'DELIVERING'] and self.sub_state == 'DWELLING' and not self.is_paused:
            if self.dwell_time_remaining > 0:
                self.dwell_time_remaining -= 1.0
                prefix = "Giao hàng tại" if self.mission_type == 'DELIVERY' else "Dừng chờ tại"
                self.status_message = f"{prefix} {self.current_goal_name or self.current_goal_id} ({int(self.dwell_time_remaining)}s còn lại)"
                if self.dwell_time_remaining <= 0:
                    self.get_logger().info(f'✅ Hết thời gian chờ tại {self.current_goal_id}. Chuyển bước tiếp theo!')
                    self.advance_to_next_waypoint()

        # 2. Kiểm tra an toàn mức pin
        if self.mission_type != 'NONE' and self.state not in ['IDLE', 'RETURNING_HOME']:
            if self.battery_percent < self.battery_low_threshold:
                self.get_logger().error(f'🔋 [BATTERY LOW] Pin chỉ còn {self.battery_percent:.1f}% (< {self.battery_low_threshold}%)! Ưu tiên về sạc khẩn cấp.')
                self.handle_low_battery()

        # 3. Publish báo cáo trạng thái lên Tablet UI
        self.publish_status()

    def publish_status(self):
        payload = {
            "state": self.state,
            "sub_state": self.sub_state,
            "mission_type": self.mission_type,
            "mission_id": self.mission_id,
            "current_waypoint_index": self.current_wp_idx,
            "total_waypoints": len(self.patrol_waypoints),
            "current_goal_id": self.current_goal_id,
            "current_goal_name": self.current_goal_name,
            "dwell_time_remaining": max(0, int(self.dwell_time_remaining)),
            "battery_percent": round(self.battery_percent, 1),
            "is_paused": self.is_paused,
            "loop": self.loop_patrol,
            "statistics": {
                "completed_count": self.completed_waypoints_count,
                "skipped_count": self.skipped_waypoints_count,
                "elapsed_time_sec": int(time.time() - self.mission_start_epoch) if self.mission_start_epoch > 0 else 0
            },
            "message": self.status_message,
            "timestamp": int(self.get_clock().now().to_msg().sec)
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def stop_robot_motion(self):
        """Dập phanh vận tốc 0 vào /cmd_vel."""
        zero_twist = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(zero_twist)
            time.sleep(0.01)

    # =========================================================================
    # XỬ LÝ LỆNH TỪ UI (COMMAND DISPATCHER)
    # =========================================================================
    def parse_command_payload(self, raw_str: str) -> dict:
        raw_clean = raw_str.strip()
        if raw_clean.startswith('{') and raw_clean.endswith('}'):
            try:
                parsed = json.loads(raw_clean)
                if isinstance(parsed, dict):
                    # Format từ rosbridge: { "command": "...", ... } hoặc { "data": "..." }
                    if "data" in parsed and isinstance(parsed["data"], dict):
                        return parsed["data"]
                    if "data" in parsed and isinstance(parsed["data"], str) and parsed["data"].strip().startswith('{'):
                        try:
                            return json.loads(parsed["data"])
                        except Exception:
                            pass
                    return parsed
            except Exception:
                pass
        return {"command": raw_clean.upper()}

    def command_callback(self, msg: String):
        payload = self.parse_command_payload(msg.data)
        cmd = str(payload.get("command") or payload.get("cmd") or payload.get("data") or "").upper().strip()

        self.get_logger().info(f'📥 [COMMAND] Nhận lệnh: "{cmd}" | State: {self.state} ({self.sub_state})')

        if cmd in ['START_DELIVERY', 'DELIVERY', 'GIAO_HANG']:
            self.handle_start_delivery(payload)
        elif cmd in ['START_WAYPOINTS', 'RUN_WAYPOINTS', 'START_MISSION', 'START_PATROL', 'RUN_ROUTE']:
            self.handle_start_patrol(payload)
        elif cmd in ['CONFIRM_DELIVERY', 'DONE', 'DA_LAY_HANG', 'CONFIRM', 'NEXT']:
            self.handle_skip_wait()
        elif cmd in ['PAUSE', 'TAM_DUNG']:
            self.handle_pause()
        elif cmd in ['RESUME', 'TIEP_TUC']:
            self.handle_resume()
        elif cmd in ['CANCEL', 'STOP']:
            self.handle_cancel()
        elif cmd == 'SKIP_WAIT':
            self.handle_skip_wait()
        elif cmd in ['SKIP_WAYPOINT', 'SKIP']:
            self.handle_skip_waypoint()
        elif cmd == 'GO_TO_GOAL':
            self.handle_go_to_single_goal(payload)
        else:
            self.get_logger().warn(f'⚠️ Lệnh không xác định: "{cmd}"')
            self.status_message = f"Unknown command: {cmd}"

    # =========================================================================
    # NGHIỆP VỤ GIAO HÀNG (DELIVERY: ĐẾN ĐIỂM -> CHỜ LẤY HÀNG -> VỀ HOME)
    # =========================================================================
    def handle_start_delivery(self, payload: dict):
        """
        Nghiệp vụ GIAO HÀNG (Delivery Mode):
        - Nhận 1 điểm (hoặc danh sách điểm bàn cần giao).
        - Robot từ HOME di chuyển tới điểm giao hàng.
        - Dừng chờ khách lấy đồ (dwell time hoặc nút xác nhận).
        - Giao hàng xong: TỰ ĐỘNG RETURN VỀ ĐIỂM HOME!
        """
        self.get_logger().info('📦 [DELIVERY] Bắt đầu thiết lập tác vụ Giao Hàng...')
        map_name = payload.get("map_name") or self.current_map_name
        self.current_map_name = map_name

        target_goal = payload.get("target_goal") or payload.get("goal_id") or payload.get("target")
        wait_sec = float(payload.get("wait_sec", payload.get("dwell_time", 30.0)))

        raw_wps = payload.get("waypoints", [])
        if not raw_wps and target_goal:
            raw_wps = [{"goal_id": str(target_goal).strip(), "wait_sec": wait_sec}]

        if not raw_wps:
            self.get_logger().error('❌ Thiếu thông tin điểm giao hàng (goal_id hoặc waypoints)!')
            self.status_message = "Error: Missing delivery target goal"
            return

        stored_goals = self.storage.load_goals(map_name) if self.storage else []
        stored_map = {str(g.get("goal_id", "")).strip(): g for g in stored_goals}
        stored_map.update({str(g.get("name", "")).strip(): g for g in stored_goals})

        parsed_waypoints = []
        for idx, item in enumerate(raw_wps):
            goal_id = ""
            name = ""
            pose_dict = None
            wp_wait = wait_sec

            if isinstance(item, str):
                goal_id = item.strip()
            elif isinstance(item, dict):
                goal_id = str(item.get("goal_id") or item.get("id") or "").strip()
                name = str(item.get("name") or "").strip()
                pose_dict = item.get("pose")
                wp_wait = float(item.get("wait_sec", item.get("dwell_time", wait_sec)))

            if not pose_dict and goal_id in stored_map:
                matched = stored_map[goal_id]
                pose_dict = matched.get("pose")
                name = name or matched.get("name") or goal_id

            if pose_dict and "x" in pose_dict and "y" in pose_dict:
                parsed_waypoints.append({
                    "goal_id": goal_id or f"WP_{idx+1}",
                    "name": name or goal_id or f"Bàn {idx+1}",
                    "pose": pose_dict,
                    "wait_sec": wp_wait,
                    "retries": 0
                })
            else:
                self.get_logger().warn(f'⚠️ Không tìm thấy tọa độ cho điểm giao hàng: "{goal_id}"')

        if not parsed_waypoints:
            self.get_logger().error('❌ Không tìm thấy tọa độ hợp lệ cho điểm giao hàng!')
            self.status_message = "Error: Invalid delivery waypoint"
            return

        self.stop_robot_motion()
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        self.mission_type = 'DELIVERY'
        self.mission_id = str(payload.get("mission_id", f"delivery_{int(time.time())}"))
        self.patrol_waypoints = parsed_waypoints
        self.current_wp_idx = 0
        self.loop_patrol = False
        self.on_finish_action = str(payload.get("on_finish", "NONE")).upper().strip()
        self.is_paused = False

        self.get_logger().info(f'📦 [DELIVERY] Sẵn sàng giao {len(self.patrol_waypoints)} điểm theo lộ trình (on_finish={self.on_finish_action}).')
        self.navigate_to_waypoint(0)

    # =========================================================================
    # KHỞI ĐỘNG VÀ QUẢN LÝ TUẦN TRA (MULTI-WAYPOINT PATROL)
    # =========================================================================
    def handle_start_patrol(self, payload: dict):
        self.get_logger().info('🚀 Bắt đầu thiết lập chu trình tuần tra Waypoint...')
        map_name = payload.get("map_name") or self.current_map_name
        self.current_map_name = map_name

        raw_wps = payload.get("waypoints", [])
        if not raw_wps or not isinstance(raw_wps, list):
            self.get_logger().error('❌ Danh sách waypoints rỗng hoặc sai định dạng!')
            self.status_message = "Error: waypoints list is empty"
            return

        # Nạp danh sách Goal đã lưu trên Map từ GoalStorage
        stored_goals = self.storage.load_goals(map_name) if self.storage else []
        stored_map = {str(g.get("goal_id", "")).strip(): g for g in stored_goals}
        stored_map.update({str(g.get("name", "")).strip(): g for g in stored_goals})

        parsed_waypoints = []
        for idx, item in enumerate(raw_wps):
            goal_id = ""
            name = ""
            pose_dict = None
            wait_sec = 30.0

            if isinstance(item, str):
                goal_id = item.strip()
            elif isinstance(item, dict):
                goal_id = str(item.get("goal_id") or item.get("id") or "").strip()
                name = str(item.get("name") or "").strip()
                pose_dict = item.get("pose")
                wait_sec = float(item.get("wait_sec", item.get("dwell_time", item.get("wait_time", 30.0))))

            # Tra cứu tọa độ nếu chưa có pose
            if not pose_dict and goal_id in stored_map:
                matched = stored_map[goal_id]
                pose_dict = matched.get("pose")
                name = name or matched.get("name") or goal_id

            if pose_dict and "x" in pose_dict and "y" in pose_dict:
                parsed_waypoints.append({
                    "goal_id": goal_id or f"WP_{idx+1}",
                    "name": name or goal_id or f"Point {idx+1}",
                    "pose": pose_dict,
                    "wait_sec": wait_sec,
                    "retries": 0
                })
            else:
                self.get_logger().warn(f'⚠️ Không tìm thấy tọa độ cho Waypoint: "{goal_id}". Điểm này sẽ bị bỏ qua.')

        if not parsed_waypoints:
            self.get_logger().error('❌ Không tìm thấy tọa độ hợp lệ cho bất kỳ waypoint nào trong map!')
            self.status_message = "Error: No valid waypoints found"
            return

        # Dọn dẹp trạng thái cũ
        self.stop_robot_motion()
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        # Cài đặt cấu hình Patrol mới
        self.mission_type = 'PATROL'
        self.mission_id = str(payload.get("mission_id", f"tour_{int(time.time())}"))
        self.patrol_waypoints = parsed_waypoints
        self.current_wp_idx = 0
        self.loop_patrol = bool(payload.get("loop", False))
        self.on_finish_action = str(payload.get("on_finish", "HOME")).upper().strip()
        self.is_paused = False

        self.get_logger().info(f'📋 Thiết lập thành công {len(self.patrol_waypoints)} điểm. Lặp: {self.loop_patrol}. Xuất phát điểm đầu tiên!')
        self.navigate_to_waypoint(0)

    def navigate_to_waypoint(self, index: int):
        """Gửi lệnh tới Nav2 để di chuyển đến điểm thứ index trong hàng đợi."""
        if index >= len(self.patrol_waypoints):
            if self.loop_patrol:
                self.get_logger().info('🔄 Hoàn thành 1 vòng tour, tự động lặp lại từ đầu!')
                self.current_wp_idx = 0
                # Reset số lần retry cho vòng mới
                for wp in self.patrol_waypoints:
                    wp["retries"] = 0
                self.navigate_to_waypoint(0)
            else:
                self.get_logger().info('🏁 Đã hoàn thành tất cả các điểm trong hành trình!')
                self.handle_patrol_finished()
            return

        self.current_wp_idx = index
        wp = self.patrol_waypoints[index]
        self.current_goal_id = wp["goal_id"]
        self.current_goal_name = wp["name"]
        pose_data = wp["pose"]

        # Chuyển pose_data thành PoseStamped
        pose_msg = self._create_pose_stamped(pose_data)
        self.saved_goal_pose = pose_msg
        self.state = 'NAVIGATING'
        self.sub_state = 'TRANSIT'
        self.status_message = f"Đang di chuyển tới {self.current_goal_name} ({index + 1}/{len(self.patrol_waypoints)})"

        self.get_logger().info(f'➡️ [{index + 1}/{len(self.patrol_waypoints)}] Di chuyển tới: "{self.current_goal_name}" (x={pose_data.get("x"):.2f}, y={pose_data.get("y"):.2f})')
        self.send_nav2_goal(pose_msg)

    def advance_to_next_waypoint(self):
        """Chuyển sang điểm kế tiếp trong hành trình."""
        next_idx = self.current_wp_idx + 1
        self.navigate_to_waypoint(next_idx)

    # =========================================================================
    # ĐIỀU KHIỂN PAUSE / RESUME KÉP (DUAL-MODE PAUSE)
    # =========================================================================
    def handle_pause(self):
        self.get_logger().info('⏸️ [PAUSE] Nhận lệnh tạm dừng tác vụ...')
        self.stop_robot_motion()

        if self.state in ['NAVIGATING', 'RUNNING'] and self.sub_state == 'TRANSIT':
            self.paused_prev_state = 'TRANSIT'
            if self.current_goal_handle:
                self.get_logger().info('Hủy Nav2 goal tạm thời để phanh xe (bảo toàn mục tiêu dở dang)...')
                self.current_goal_handle.cancel_goal_async()
            self.status_message = f"Tạm dừng khi đang đi tới {self.current_goal_name}"

        elif self.state in ['WAITING', 'DELIVERING'] and self.sub_state == 'DWELLING':
            self.paused_prev_state = 'DWELLING'
            self.status_message = f"Tạm dừng khi đang chờ tại {self.current_goal_name} ({int(self.dwell_time_remaining)}s còn lại)"

        self.is_paused = True
        self.state = 'PAUSED'
        self.publish_status()

    def handle_resume(self):
        self.get_logger().info('▶️ [RESUME] Nhận lệnh tiếp tục tác vụ...')
        if not self.is_paused or self.state != 'PAUSED':
            self.get_logger().warn(f'⚠️ Không thể RESUME khi trạng thái là "{self.state}".')
            return

        self.is_paused = False

        if self.paused_prev_state == 'TRANSIT':
            self.state = 'NAVIGATING'
            self.sub_state = 'TRANSIT'
            self.status_message = f"Tiếp tục di chuyển tới {self.current_goal_name}"
            if self.saved_goal_pose:
                self.get_logger().info(f'Gửi lại tọa độ {self.current_goal_name} cho Nav2...')
                self.send_nav2_goal(self.saved_goal_pose)
            else:
                self.advance_to_next_waypoint()

        elif self.paused_prev_state == 'DWELLING':
            self.state = 'DELIVERING' if self.mission_type == 'DELIVERY' else 'WAITING'
            self.sub_state = 'DWELLING'
            self.status_message = f"Tiếp tục chờ tại {self.current_goal_name} ({int(self.dwell_time_remaining)}s còn lại)"
            self.get_logger().info(f'Tiếp tục đếm nốt {self.dwell_time_remaining}s chờ tại điểm.')

        self.publish_status()

    def handle_skip_wait(self):
        """Khách bấm đã lấy hàng xong (hoặc ấn nút trên UI) để xe đi tiếp ngay tới bàn tiếp theo hoặc về HOME."""
        if (self.state in ['WAITING', 'DELIVERING'] and self.sub_state == 'DWELLING') or (self.state == 'PAUSED' and self.paused_prev_state == 'DWELLING'):
            self.get_logger().info('⏩ [CONFIRM/DA_LAY_HANG] Khách đã lấy hàng xong! Bỏ qua thời gian chờ còn lại, đi tiếp...')
            self.is_paused = False
            self.dwell_time_remaining = 0.0
            self.advance_to_next_waypoint()
        else:
            self.get_logger().warn(f'⚠️ Lệnh CONFIRM chỉ có hiệu lực khi robot đang dừng chờ tại điểm (Hiện tại: State={self.state}, SubState={self.sub_state}).')

    def handle_skip_waypoint(self):
        """Bỏ qua điểm hiện tại khi đang đi đường."""
        self.get_logger().info('⏭️ [SKIP WAYPOINT] Bỏ qua waypoint hiện tại!')
        self.stop_robot_motion()
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.advance_to_next_waypoint()

    def handle_cancel(self):
        """Hủy toàn bộ tour và đưa robot về nghỉ."""
        self.get_logger().info('⏹️ [CANCEL] Hủy toàn bộ nhiệm vụ hiện tại.')
        self.stop_robot_motion()
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        self.state = 'IDLE'
        self.sub_state = 'NONE'
        self.mission_type = 'NONE'
        self.patrol_waypoints = []
        self.saved_goal_pose = None
        self.is_paused = False
        self.status_message = "Nhiệm vụ đã bị hủy. Robot ở chế độ IDLE."
        self.publish_status()

    # =========================================================================
    # KẾT THÚC TOUR / HẾT GIỜ / PIN YẾU -> VỀ TRẠM (HOME / CHARGE)
    # =========================================================================
    def handle_patrol_finished(self):
        self.status_message = "Đã hoàn thành toàn bộ hành trình."
        if self.on_finish_action in ['HOME', 'CHARGING_DOCK', 'CHARGE']:
            self.get_logger().info(f'🏠 Tour hoàn tất -> Tự động quay về trạm ({self.on_finish_action})...')
            self.navigate_to_home_or_dock()
        else:
            self.get_logger().info('🏁 Tour hoàn tất -> Robot chuyển về trạng thái IDLE.')
            self.state = 'IDLE'
            self.sub_state = 'NONE'
            self.mission_type = 'NONE'
            self.status_message = "Đã hoàn thành toàn bộ lộ trình. Robot sẵn sàng."
            self.publish_status()

    def handle_low_battery(self):
        self.stop_robot_motion()
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        self.status_message = f"Pin yếu ({self.battery_percent:.1f}%). Khẩn cấp về trạm sạc."
        self.navigate_to_home_or_dock()

    def navigate_to_home_or_dock(self):
        """Tìm tọa độ HOME hoặc CHARGING_DOCK và điều hướng về."""
        self.state = 'RETURNING_HOME'
        self.sub_state = 'TRANSIT'
        self.mission_type = 'PATROL'

        home_pose = None
        if self.storage:
            stored_goals = self.storage.load_goals(self.current_map_name)
            for g in stored_goals:
                gid = str(g.get("goal_id", "")).upper()
                gname = str(g.get("name", "")).upper()
                if gid in ['HOME', 'CHARGING_DOCK', 'DOCK', 'TRAM_SAC'] or gname in ['HOME', 'CHARGING_DOCK', 'TRẠM SẠC']:
                    home_pose = g.get("pose")
                    self.current_goal_id = g.get("goal_id", "HOME")
                    self.current_goal_name = g.get("name", "Trạm Sạc")
                    break

        if home_pose:
            self.get_logger().info(f'🏠 Tìm thấy vị trí trạm: "{self.current_goal_name}". Điều hướng về trạm...')
            pose_msg = self._create_pose_stamped(home_pose)
            self.saved_goal_pose = pose_msg
            self.send_nav2_goal(pose_msg)
        else:
            self.get_logger().warn('⚠️ Không tìm thấy điểm HOME/CHARGING_DOCK trong map. Dừng robot tại chỗ.')
            self.state = 'COMPLETED'
            self.sub_state = 'NONE'
            self.mission_type = 'NONE'
            self.publish_status()

    # =========================================================================
    # ĐIỀU HƯỚNG ĐIỂM ĐƠN LẺ (SINGLE GOAL / COMPATIBILITY)
    # =========================================================================
    def single_goal_callback(self, msg: PoseStamped):
        """Nhận goal đơn lẻ trực tiếp từ RViz hoặc topic /goal_pose."""
        self.get_logger().info(f'📍 Nhận điểm đơn lẻ qua /goal_pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.stop_robot_motion()
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()

        self.mission_type = 'SINGLE'
        self.current_goal_id = "CUSTOM"
        self.current_goal_name = "Custom Target"
        self.saved_goal_pose = msg
        self.state = 'RUNNING'
        self.sub_state = 'TRANSIT'
        self.is_paused = False
        self.status_message = "Đang đi tới tọa độ chỉ định"
        self.send_nav2_goal(msg)

    def handle_go_to_single_goal(self, payload: dict):
        map_name = payload.get("map_name") or self.current_map_name
        goal_id = str(payload.get("goal_id") or payload.get("name") or "").strip()

        pose_data = payload.get("pose")
        if not pose_data and self.storage:
            stored_goals = self.storage.load_goals(map_name)
            for g in stored_goals:
                if str(g.get("goal_id", "")).strip() == goal_id or str(g.get("name", "")).strip() == goal_id:
                    pose_data = g.get("pose")
                    break

        if not pose_data:
            self.get_logger().error(f'Không tìm thấy tọa độ cho goal "{goal_id}"')
            self.status_message = f"Goal not found: {goal_id}"
            return

        pose_msg = self._create_pose_stamped(pose_data)
        self.single_goal_callback(pose_msg)

    # =========================================================================
    # NAV2 ACTION CLIENT & KẾT QUẢ / RETRY
    # =========================================================================
    def send_nav2_goal(self, pose: PoseStamped):
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('❌ Nav2 Action Server "navigate_to_pose" không phản hồi!')
            self.status_message = "Nav2 Action Server not available"
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        future = self.nav2_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav2_goal_response_cb)

    def nav2_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 từ chối Goal!')
            self.handle_nav_failure()
            return

        self.get_logger().info('✅ Nav2 đã tiếp nhận Goal. Robot bắt đầu di chuyển...')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_cb)

    def nav2_result_cb(self, future):
        result = future.result()
        status = result.status
        self.current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'🎯 ĐÃ ĐẾN NƠI: Robot đã tới mục tiêu "{self.current_goal_name}"!')

            if self.mission_type in ['PATROL', 'DELIVERY']:
                if self.state == 'RETURNING_HOME':
                    self.get_logger().info('🏁 Robot đã về đến Trạm sạc/Home an toàn! Nhiệm vụ kết thúc.')
                    self.state = 'IDLE'
                    self.sub_state = 'NONE'
                    self.mission_type = 'NONE'
                    self.status_message = "Robot đã về trạm an toàn. Đã hoàn thành nhiệm vụ."
                    self.publish_status()
                    return

                # Robot tới một waypoint trong lộ trình
                self.completed_waypoints_count += 1
                current_wp = self.patrol_waypoints[self.current_wp_idx]
                wait_sec = float(current_wp.get("wait_sec", 0.0))

                if wait_sec > 0:
                    state_name = 'DELIVERING' if self.mission_type == 'DELIVERY' else 'WAITING'
                    prefix = "Giao hàng tại" if self.mission_type == 'DELIVERY' else "Dừng chờ tại"
                    self.get_logger().info(f'⏳ {prefix} {self.current_goal_name}: bắt đầu đếm thời gian chờ {wait_sec}s...')
                    self.stop_robot_motion()
                    self.state = state_name
                    self.sub_state = 'DWELLING'
                    self.dwell_time_remaining = wait_sec
                    self.status_message = f"{prefix} {self.current_goal_name} ({int(wait_sec)}s)"
                else:
                    self.get_logger().info('Không có thời gian chờ, đi tiếp bước kế tiếp.')
                    self.advance_to_next_waypoint()

            else:  # Single Goal
                self.state = 'COMPLETED'
                self.sub_state = 'NONE'
                self.mission_type = 'NONE'
                self.saved_goal_pose = None
                self.status_message = "Đã đến đích thành công"

        elif status == GoalStatus.STATUS_CANCELED:
            if self.is_paused:
                self.get_logger().info('🛑 Goal Nav2 bị hủy do nhận lệnh PAUSE (Bảo toàn đích đến).')
            else:
                self.get_logger().info('🛑 Goal Nav2 bị hủy hoàn toàn.')
                if self.mission_type == 'SINGLE':
                    self.state = 'IDLE'
                    self.sub_state = 'NONE'

        else:
            self.get_logger().warn(f'⚠️ Nav2 không thể tới đích (Status: {status})!')
            self.handle_nav_failure()

    def handle_nav_failure(self):
        """Xử lý khi Nav2 gặp vật cản hoặc thất bại."""
        self.stop_robot_motion()

        if self.mission_type in ['PATROL', 'DELIVERY'] and self.current_wp_idx < len(self.patrol_waypoints):
            current_wp = self.patrol_waypoints[self.current_wp_idx]
            current_wp["retries"] = current_wp.get("retries", 0) + 1

            if current_wp["retries"] <= self.max_retries_per_wp:
                self.get_logger().warn(f'🔄 Thử lại lần {current_wp["retries"]}/{self.max_retries_per_wp} cho điểm "{self.current_goal_name}" sau 2 giây...')
                self.status_message = f"Đường bị chặn, đang thử lại lần {current_wp['retries']}..."
                time.sleep(2.0)
                if self.saved_goal_pose:
                    self.send_nav2_goal(self.saved_goal_pose)
            else:
                self.get_logger().error(f'❌ Đã thử lại {self.max_retries_per_wp} lần nhưng vẫn bị chặn tại "{self.current_goal_name}". Bỏ qua điểm này để đi tiếp!')
                self.status_message = f"Bỏ qua {self.current_goal_name} do bị kẹt"
                self.skipped_waypoints_count += 1
                self.advance_to_next_waypoint()
        else:
            self.state = 'IDLE'
            self.sub_state = 'NONE'
            self.status_message = "Di chuyển thất bại do gặp vật cản"

    # =========================================================================
    # TIỆN ÍCH BIẾN ĐỔI POSE
    # =========================================================================
    def _create_pose_stamped(self, pose_dict: dict) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose_data_x := pose_dict.get("x", 0.0))
        msg.pose.position.y = float(pose_data_y := pose_dict.get("y", 0.0))
        msg.pose.position.z = 0.0

        theta = float(pose_dict.get("theta", 0.0))
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TaskControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
