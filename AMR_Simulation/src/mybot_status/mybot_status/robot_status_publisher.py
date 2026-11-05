#!/usr/bin/env python3
"""
robot_status_publisher.py
-------------------------
Node ROS 2 Trung Tâm Báo Cáo Trạng Thái Toàn Diện (Unified Robot Status Hub):
  1. Tọa độ thực tế & góc hướng đầu xe: pose {x, y, theta} (từ AMCL & Odometry)
  2. Mức pin: battery_percent (từ cảm biến pin hoặc giả lập)
  3. Trạng thái vận tốc chuyển động: is_moving, linear_speed, angular_speed (từ /cmd_vel)
  4. Trạng thái nghiệp vụ & Giao hàng (tích hợp từ /task_controller/status):
     - state (idle, navigating, waiting, delivering, returning_home, paused)
     - dwell_time_remaining (đếm ngược chờ khách lấy đồ)
     - mission_time_remaining (đếm ngược 2 giờ ca làm việc)
     - current_goal_name (Bàn 1, Bàn 2)
     - statistics (số điểm đã giao, số điểm bị kẹt)
  5. Xuất bản đồng thời lên:
     - /robot_status: Topic chuẩn toàn diện cho Tablet UI
     - /robot_status_simple: Topic tương thích ngược
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_action_status_default,
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy
)
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from action_msgs.msg import GoalStatusArray, GoalStatus


class RobotStatusPublisher(Node):

    def __init__(self):
        super().__init__('robot_status_publisher')

        # ----------------- Parameters an toàn -----------------
        self._declare_param_safe('publish_period', 0.5)  # 2.0 Hz
        self._declare_param_safe('simulated_battery', 100.0)
        self._declare_param_safe('use_sim_time', True)

        period = float(self.get_parameter('publish_period').value)
        self.simulated_battery = float(self.get_parameter('simulated_battery').value)

        # ----------------- Biến Trạng Thái Xe -----------------
        self.status = "idle"
        self.pose_x = None
        self.pose_y = None
        self.pose_theta = 0.0
        self.is_moving = False
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.battery_percent = self.simulated_battery

        # ----------------- Biến Nghiệp Vụ Task Controller -----------------
        self.task_data = {
            "state": "IDLE",
            "sub_state": "NONE",
            "mission_type": "NONE",
            "mission_id": "",
            "current_waypoint_index": 0,
            "total_waypoints": 0,
            "current_goal_id": None,
            "current_goal_name": None,
            "dwell_time_remaining": 0,
            "is_paused": False,
            "statistics": {"completed_count": 0, "skipped_count": 0, "elapsed_time_sec": 0},
            "message": "Robot đang ở trạm nghỉ (IDLE)"
        }
        self.last_task_update_time = 0.0

        # Nav2 internal status tracking
        self.nav2_status = "idle"
        self.status_hold_until = 0.0

        # ----------------- Publishers -----------------
        # Master Topic duy nhất cho Tablet UI
        self.pub_status_master = self.create_publisher(String, '/robot_status', 10)
        # Topic tương thích ngược
        self.pub_status_simple = self.create_publisher(String, '/robot_status_simple', 10)

        # ----------------- Subscribers -----------------
        # 1. Tọa độ từ AMCL
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_cb,
            10
        )

        # 2. Tọa độ từ Odometry (fallback)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )

        # 3. Vận tốc tức thời từ /cmd_vel
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )

        # 4. Trạng thái nhiệm vụ từ mybot_task_controller
        self.create_subscription(
            String,
            '/task_controller/status',
            self.task_controller_status_cb,
            10
        )

        # 5. Theo dõi Pin
        self.create_subscription(
            BatteryState,
            '/battery_status',
            self.battery_cb,
            10
        )
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_cb,
            10
        )
        self.create_subscription(
            Float32,
            '/battery_percent',
            self.battery_float_cb,
            10
        )

        # 6. Theo dõi Nav2 Action Status
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav2_status_cb,
            qos_profile_action_status_default
        )

        # Timer định kỳ (2.0 Hz)
        self.timer = self.create_timer(period, self.publish_unified_status)

        self.get_logger().info('======================================================================')
        self.get_logger().info('🚀 Unified Robot Status Hub (mybot_status) running!')
        self.get_logger().info('📡 Publishing master status to: /robot_status & /robot_status_simple')
        self.get_logger().info('📥 Aggregating AMCL, Odom, Battery, cmd_vel & Task Controller Status')
        self.get_logger().info('======================================================================')

    def _declare_param_safe(self, name: str, default_val):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_val)

    def _quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # =========================================================================
    # CALLBACKS THU THẬP TỌA ĐỘ VÀ VẬN TỐC
    # =========================================================================
    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose_x = p.x
        self.pose_y = p.y
        self.pose_theta = self._quaternion_to_yaw(q)

    def odom_cb(self, msg: Odometry):
        if self.pose_x is None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.pose_x = p.x
            self.pose_y = p.y
            self.pose_theta = self._quaternion_to_yaw(q)

    def cmd_vel_cb(self, msg: Twist):
        self.current_linear_speed = math.hypot(msg.linear.x, msg.linear.y)
        self.current_angular_speed = abs(msg.angular.z)
        self.is_moving = (self.current_linear_speed > 0.01 or self.current_angular_speed > 0.01)

    # =========================================================================
    # CALLBACK NHẬN DỮ LIỆU TỪ TASK CONTROLLER
    # =========================================================================
    def task_controller_status_cb(self, msg: String):
        try:
            parsed = json.loads(msg.data)
            if isinstance(parsed, dict):
                self.task_data.update(parsed)
                self.last_task_update_time = time.time()
        except Exception:
            pass

    # =========================================================================
    # CALLBACK PIN
    # =========================================================================
    def battery_cb(self, msg: BatteryState):
        if msg.percentage is not None and msg.percentage > 0.0:
            self.battery_percent = msg.percentage if msg.percentage > 1.0 else (msg.percentage * 100.0)

    def battery_float_cb(self, msg: Float32):
        self.battery_percent = float(msg.data)

    # =========================================================================
    # CALLBACK NAV2 STATUS
    # =========================================================================
    def nav2_status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        nav_status = latest.status
        if nav_status == GoalStatus.STATUS_ACCEPTED:
            self.nav2_status = "preparing"
        elif nav_status == GoalStatus.STATUS_EXECUTING:
            self.nav2_status = "moving"
        elif nav_status == GoalStatus.STATUS_SUCCEEDED:
            self.nav2_status = "arrived"
            self.status_hold_until = time.time() + 2.0
        elif nav_status == GoalStatus.STATUS_CANCELED:
            self.nav2_status = "canceled"
            self.status_hold_until = time.time() + 2.0
        elif nav_status == GoalStatus.STATUS_ABORTED:
            self.nav2_status = "error"
            self.status_hold_until = time.time() + 2.0

    # =========================================================================
    # XUẤT BẢN STATUS TỔNG HỢP (UNIFIED STATUS)
    # =========================================================================
    def publish_unified_status(self):
        now = time.time()

        # Xác định trạng thái tổng thể (Global System Status)
        task_active = (now - self.last_task_update_time < 3.0) and (self.task_data.get("mission_type", "NONE") != "NONE")
        
        if task_active:
            # Ưu tiên lấy trạng thái từ Task Controller (NAVIGATING, WAITING, DELIVERING, RETURNING_HOME, PAUSED)
            display_status = str(self.task_data.get("state", "IDLE")).lower()
        else:
            if self.status_hold_until > 0 and now < self.status_hold_until:
                display_status = self.nav2_status
            elif self.is_moving:
                display_status = "moving"
            else:
                display_status = "idle"

        pose_data = None
        if self.pose_x is not None and self.pose_y is not None:
            pose_data = {
                "x": round(self.pose_x, 3),
                "y": round(self.pose_y, 3),
                "theta": round(self.pose_theta, 3)
            }

        # 1. Gói Master Status Đầy Đủ (Chuẩn cho UI Tablet)
        master_payload = {
            "status": display_status,
            "pose": pose_data,
            "battery_percent": round(self.battery_percent, 1),
            "is_moving": self.is_moving,
            "current_goal": self.task_data.get("current_goal_name") or self.task_data.get("current_goal_id"),
            "task": {
                "state": self.task_data.get("state", "IDLE"),
                "sub_state": self.task_data.get("sub_state", "NONE"),
                "mission_type": self.task_data.get("mission_type", "NONE"),
                "mission_id": self.task_data.get("mission_id", ""),
                "current_waypoint_index": self.task_data.get("current_waypoint_index", 0),
                "total_waypoints": self.task_data.get("total_waypoints", 0),
                "current_goal_id": self.task_data.get("current_goal_id"),
                "current_goal_name": self.task_data.get("current_goal_name"),
                "dwell_time_remaining": self.task_data.get("dwell_time_remaining", 0),
                "is_paused": self.task_data.get("is_paused", False),
                "statistics": self.task_data.get("statistics", {})
            },
            "message": self.task_data.get("message", f"Robot đang ở trạng thái {display_status}"),
            "timestamp": int(self.get_clock().now().to_msg().sec)
        }

        # 2. Gói Simple Status (Giữ nguyên 100% trường cũ & tích hợp thêm /task_controller/status)
        simple_payload = {
            "status": display_status,
            "current_goal": master_payload["current_goal"],
            "battery_percent": master_payload["battery_percent"],
            "pose": {"x": round(self.pose_x, 2), "y": round(self.pose_y, 2)} if self.pose_x is not None else None,
            # Thêm trực tiếp các trường nghiệp vụ từ /task_controller/status:
            "task_state": self.task_data.get("state", "IDLE"),
            "dwell_time_remaining": self.task_data.get("dwell_time_remaining", 0),
            "current_waypoint_index": self.task_data.get("current_waypoint_index", 0),
            "total_waypoints": self.task_data.get("total_waypoints", 0),
            "is_paused": self.task_data.get("is_paused", False),
            "statistics": self.task_data.get("statistics", {}),
            "task": self.task_data,
            "message": self.task_data.get("message", f"Robot đang ở trạng thái {display_status}")
        }

        msg_master = String()
        msg_master.data = json.dumps(master_payload, ensure_ascii=False)
        self.pub_status_master.publish(msg_master)

        msg_simple = String()
        msg_simple.data = json.dumps(simple_payload, ensure_ascii=False)
        self.pub_status_simple.publish(msg_simple)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
