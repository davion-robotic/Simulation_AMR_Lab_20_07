#!/usr/bin/env python3
"""
mode_manager_node.py: ROS 2 node điều phối chế độ hệ thống (MAPPING, NAVIGATION, IDLE)
kết nối trực tiếp với Tablet/Web UI thông qua WebSocket ROSBridge.
"""

import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


class ModeManagerNode(Node):
    """
    Quản lý chế độ hoạt động của AMR:
      - MAPPING: Tự động chạy SLAM Mapping (mapping.launch.py)
      - NAVIGATION: Tự động chạy Nav2 Navigation (navigation2.launch.py)
      - IDLE: Dừng các tiến trình và đưa robot về trạng thái nghỉ
    """

    def __init__(self):
        super().__init__('mode_manager_node')

        # Khai báo parameters an toàn (tránh ParameterAlreadyDeclaredException)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if not self.has_parameter('default_map'):
            self.declare_parameter('default_map', 'map_lab20')
        if not self.has_parameter('set_mode_topic'):
            self.declare_parameter('set_mode_topic', '/system_mode/set_mode')
        if not self.has_parameter('state_topic'):
            self.declare_parameter('state_topic', '/system_mode/state')
        if not self.has_parameter('cmd_vel_topic'):
            self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.default_map = self.get_parameter('default_map').get_parameter_value().string_value
        set_mode_topic = self.get_parameter('set_mode_topic').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # Trạng thái hiện tại: 'IDLE', 'SWITCHING', 'MAPPING', 'NAVIGATION'
        self.current_state = 'IDLE'
        self.active_mode = 'IDLE'
        self.current_map_name = self.default_map

        # Tiến trình nền (Subprocesses)
        self.current_process = None
        self.lock = threading.Lock()

        # Publisher / Subscriber
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.set_mode_sub = self.create_subscription(
            String,
            set_mode_topic,
            self.on_set_mode_received,
            10
        )

        # Timer publish trạng thái định kỳ 1 Hz lên WebSocket UI
        self.timer = self.create_timer(1.0, self.publish_periodic_state)

        self.get_logger().info('====================================================')
        self.get_logger().info('🚀 System Mode Manager Node initialized successfully!')
        self.get_logger().info(f'📡 Listening for mode requests on: {set_mode_topic}')
        self.get_logger().info(f'📊 Publishing state feedback to:   {state_topic}')
        self.get_logger().info(f'🗺️  Default Navigation Map:        {self.default_map}')
        self.get_logger().info(f'⏱️  use_sim_time:                   {self.use_sim_time}')
        self.get_logger().info('====================================================')

        # Publish trạng thái ban đầu
        self.publish_state('IDLE')

    def publish_state(self, state: str):
        """Publish trạng thái lên topic /system_mode/state."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f'📢 [State Update] -> {state}')

    def publish_periodic_state(self):
        """Phát lại trạng thái định kỳ 1 Hz để Tablet UI luôn giữ đồng bộ."""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    def stop_robot_motion(self):
        """Gửi lệnh dừng khẩn cấp vào /cmd_vel."""
        zero_twist = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(zero_twist)
            time.sleep(0.02)

    def parse_payload(self, raw_str: str):
        """Phân tích dữ liệu thô nhận từ WebSocket hoặc CLI."""
        raw_clean = raw_str.strip()
        target_mode = ""
        map_name = self.current_map_name

        if raw_clean.startswith('{') and raw_clean.endswith('}'):
            try:
                data = json.loads(raw_clean)
                # Dữ liệu từ roslibjs WebSocket: { "data": "MAPPING" }
                if 'data' in data:
                    target_mode = str(data['data']).strip().upper()
                elif 'mode' in data:
                    target_mode = str(data['mode']).strip().upper()
                elif 'command' in data:
                    target_mode = str(data['command']).strip().upper()

                if 'map_name' in data and data['map_name']:
                    map_name = str(data['map_name']).strip()
                elif 'map' in data and data['map']:
                    map_name = str(data['map']).strip()
            except Exception as e:
                self.get_logger().error(f'Error parsing JSON payload: {e}')
        else:
            target_mode = raw_clean.upper()

        return target_mode, map_name

    def on_set_mode_received(self, msg: String):
        """Callback khi nhận yêu cầu chuyển mode từ Tablet UI."""
        target_mode, map_name = self.parse_payload(msg.data)

        self.get_logger().info('----------------------------------------------------')
        self.get_logger().info(f'📥 [UI Request] Nhận lệnh chuyển Mode: "{target_mode}"')
        self.get_logger().info(f'   ├─ Dữ liệu gốc: {msg.data}')
        self.get_logger().info(f'   └─ Trạng thái hiện tại: {self.current_state}')

        valid_modes = ['MAPPING', 'LOCALIZATION', 'NAVIGATION', 'IDLE']
        if target_mode not in valid_modes:
            self.get_logger().warn(f'⚠️ Chế độ không hợp lệ: "{target_mode}". Các chế độ hợp lệ: {valid_modes}')
            return

        if target_mode == self.active_mode and self.current_state == target_mode:
            self.get_logger().info(f'ℹ️ Hệ thống đã ở chế độ {target_mode}. Bỏ qua.')
            self.publish_state(self.current_state)
            return

        # Thực thi chuyển mode trong luồng riêng để không block ROS spin
        threading.Thread(
            target=self._execute_mode_switch,
            args=(target_mode, map_name),
            daemon=True
        ).start()

    def _execute_mode_switch(self, target_mode: str, map_name: str):
        """Xử lý quá trình chuyển đổi giữa các mode một cách an toàn."""
        with self.lock:
            # 1. Báo trạng thái SWITCHING cho UI hiển thị loading
            self.current_state = 'SWITCHING'
            self.publish_state('SWITCHING')

            # 2. Dừng robot chuyển động
            self.stop_robot_motion()

            # 3. Dừng tiến trình cũ nếu đang chạy
            self._terminate_current_process()

            # 4. Bật chế độ mới
            success = False
            if target_mode == 'IDLE':
                self.active_mode = 'IDLE'
                self.current_state = 'IDLE'
                success = True
                self.get_logger().info('✅ Chuyển về trạng thái IDLE thành công.')

            elif target_mode == 'MAPPING':
                success = self._start_mapping_mode()
                if success:
                    self.active_mode = 'MAPPING'
                    self.current_state = 'MAPPING'
                    self.get_logger().info('✅ Đã bật chế độ MAPPING thành công.')
                else:
                    self.active_mode = 'IDLE'
                    self.current_state = 'IDLE'
                    self.get_logger().error('❌ Bật chế độ MAPPING thất bại.')

            elif target_mode == 'LOCALIZATION':
                self.current_map_name = map_name
                success = self._start_localization_mode(map_name)
                if success:
                    self.active_mode = 'LOCALIZATION'
                    self.current_state = 'LOCALIZATION'
                    self.get_logger().info(f'✅ Đã bật chế độ STANDALONE LOCALIZATION với map "{map_name}" thành công.')
                else:
                    self.active_mode = 'IDLE'
                    self.current_state = 'IDLE'
                    self.get_logger().error('❌ Bật chế độ LOCALIZATION thất bại.')

            elif target_mode == 'NAVIGATION':
                self.current_map_name = map_name
                success = self._start_navigation_mode(map_name)
                if success:
                    self.active_mode = 'NAVIGATION'
                    self.current_state = 'NAVIGATION'
                    self.get_logger().info(f'✅ Đã bật chế độ NAVIGATION với map "{map_name}" thành công.')
                else:
                    self.active_mode = 'IDLE'
                    self.current_state = 'IDLE'
                    self.get_logger().error('❌ Bật chế độ NAVIGATION thất bại.')

            # 5. Publish trạng thái cuối cùng cho UI mở khóa nút
            self.publish_state(self.current_state)

    def _terminate_current_process(self):
        """Hủy tiến trình nền cùng toàn bộ nhóm tiến trình con của nó."""
        if self.current_process is not None:
            self.get_logger().info('🛑 Đang dừng tiến trình nền trước đó...')
            try:
                pgid = os.getpgid(self.current_process.pid)
                os.killpg(pgid, signal.SIGINT)
                self.current_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('⚠️ Tiến trình chưa dừng kịp, gửi tín hiệu SIGTERM / SIGKILL...')
                try:
                    os.killpg(pgid, signal.SIGKILL)
                    self.current_process.wait(timeout=2.0)
                except Exception:
                    pass
            except Exception as e:
                self.get_logger().error(f'Lỗi khi dừng tiến trình: {e}')

            self.current_process = None
            time.sleep(0.5)

    def _start_mapping_mode(self) -> bool:
        """Khởi chạy launch file Mapping (SLAM Toolbox)."""
        try:
            sim_time_str = 'true' if self.use_sim_time else 'false'
            cmd = [
                'ros2', 'launch', 'mybot_navigation2', 'mapping.launch.py',
                f'use_sim_time:={sim_time_str}'
            ]
            self.get_logger().info(f'🚀 Khởi chạy: {" ".join(cmd)}')
            self.current_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Tạo process group mới để kill sạch
            )
            time.sleep(2.0)
            return self.current_process.poll() is None
        except Exception as e:
            self.get_logger().error(f'Không thể khởi chạy Mapping: {e}')
            return False

    def _resolve_map_path(self, map_name: str) -> str:
        """Tìm đường dẫn file yaml của bản đồ."""
        clean_name = map_name.replace('.yaml', '').replace('.pgm', '')
        
        # Thử tìm trong package share directory
        candidates = []
        try:
            pkg_share = get_package_share_directory('mybot_navigation2')
            candidates.append(os.path.join(pkg_share, 'map', f'{clean_name}.yaml'))
        except Exception:
            pass

        # Thử tìm trong thư mục source
        src_map_dir = '/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_navigation2/map'
        candidates.append(os.path.join(src_map_dir, f'{clean_name}.yaml'))
        candidates.append(os.path.join(src_map_dir, 'map_lab20.yaml'))
        candidates.append(os.path.join(src_map_dir, 'my_map.yaml'))

        for p in candidates:
            if os.path.exists(p):
                return p

        return candidates[0]

    def _start_localization_mode(self, map_name: str) -> bool:
        """Khởi chạy chế độ định vị độc lập (Standalone AMCL + Map Server) để setup goal, vẽ tường ảo, xóa noise."""
        try:
            map_path = self._resolve_map_path(map_name)
            self.get_logger().info(f'🗺️ [LOCALIZATION] Nạp bản đồ: {map_path}')

            sim_time_str = 'true' if self.use_sim_time else 'false'
            cmd = [
                'ros2', 'launch', 'nav2_bringup', 'localization_launch.py',
                f'map:={map_path}',
                f'use_sim_time:={sim_time_str}',
                'use_composition:=false'
            ]
            self.get_logger().info(f'🚀 Khởi chạy: {" ".join(cmd)}')
            self.current_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            time.sleep(2.5)
            return self.current_process.poll() is None
        except Exception as e:
            self.get_logger().error(f'Không thể khởi chạy Standalone Localization: {e}')
            return False

    def _start_navigation_mode(self, map_name: str) -> bool:
        """Khởi chạy launch file Nav2 Navigation."""
        try:
            map_path = self._resolve_map_path(map_name)
            self.get_logger().info(f'🗺️ Sử dụng bản đồ: {map_path}')

            sim_time_str = 'true' if self.use_sim_time else 'false'
            cmd = [
                'ros2', 'launch', 'mybot_navigation2', 'navigation2.launch.py',
                f'map:={map_path}',
                f'use_sim_time:={sim_time_str}'
            ]
            self.get_logger().info(f'🚀 Khởi chạy: {" ".join(cmd)}')
            self.current_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            time.sleep(3.0)
            return self.current_process.poll() is None
        except Exception as e:
            self.get_logger().error(f'Không thể khởi chạy Navigation: {e}')
            return False

    def destroy_node(self):
        """Dọn dẹp sạch tiến trình khi node bị tắt."""
        self._terminate_current_process()
        self.stop_robot_motion()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

