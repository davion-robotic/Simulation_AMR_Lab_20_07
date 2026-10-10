#!/usr/bin/env python3
"""
robot_status_publisher.py
-------------------------
Node ROS 2 theo dõi trạng thái Nav2, vị trí robot và pin,
publish JSON định kỳ mỗi 0.5s lên topic /robot_status_simple cho UI/Web (WebSocket).

Đặc tả JSON (msg.data):
{
  "status": "idle" | "preparing" | "moving" | "arrived" | "canceling" | "canceled" | "error" | "charging",
  "current_goal": "P2" | null,
  "battery_percent": 100.0 | null,
  "pose": {"x": 3.2, "y": 1.5} | null
}
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
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class RobotStatusPublisher(Node):

    def __init__(self):
        super().__init__('robot_status_publisher')

        # Parameters
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('simulated_battery', 100.0)

        period = self.get_parameter('publish_period').value
        self.simulated_battery = self.get_parameter('simulated_battery').value

        # Robot State Variables
        self.status = "idle"
        self.current_goal = None
        self.battery_percent = self.simulated_battery
        self.pose_x = None
        self.pose_y = None

        # Target Goal coordinates
        self.target_x = None
        self.target_y = None

        # Internal state tracking
        self.last_status_change_time = time.time()
        self.last_moving_time = time.time()
        self.status_hold_until = 0.0

        # Publisher
        self.publisher_ = self.create_publisher(String, '/robot_status_simple', 10)
        self.timer = self.create_timer(period, self.publish_status)

        # QoS Profiles
        transient_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Action Client to Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        # 1. Pose từ AMCL
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_cb,
            10
        )

        # 2. Pose từ Odometry (fallback)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )

        # 3. Vận tốc điều khiển cmd_vel (xác nhận chuyển động tức thì)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )

        # 4. Theo dõi status từ Nav2 NavigateToPose & NavigateThroughPoses Action Server với TRANSIENT_LOCAL QoS
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav2_status_cb,
            qos_profile_action_status_default
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_through_poses/_action/status',
            self.nav2_status_cb,
            qos_profile_action_status_default
        )

        # 5. Theo dõi Goal Pose (Topic từ RViz / Node gửi goal)
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_cb,
            transient_qos
        )
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_cb,
            10
        )

        # 6. Theo dõi tên Goal riêng biệt (nếu UI gửi tên "P1", "P2")
        self.create_subscription(
            String,
            '/current_goal_name',
            self.goal_name_cb,
            10
        )

        # 7. Theo dõi Pin
        self.create_subscription(
            BatteryState,
            '/battery_status',
            self.battery_cb,
            10
        )
        self.create_subscription(
            Float32,
            '/battery_percent',
            self.battery_float_cb,
            10
        )

        self.get_logger().info('Robot Status Publisher Running! Monitoring Nav2, /cmd_vel & /goal_pose')

    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.check_goal_arrival()

    def odom_cb(self, msg: Odometry):
        if self.pose_x is None:
            self.pose_x = msg.pose.pose.position.x
            self.pose_y = msg.pose.pose.position.y
            self.check_goal_arrival()

    def cmd_vel_cb(self, msg: Twist):
        linear_speed = math.hypot(msg.linear.x, msg.linear.y)
        angular_speed = abs(msg.angular.z)

        if linear_speed > 0.01 or angular_speed > 0.01:
            self.last_moving_time = time.time()
            if self.status in ["idle", "preparing"]:
                self.set_status("moving")

    def check_goal_arrival(self):
        # Nếu đang moving/preparing mà khoảng cách tới target goal nhỏ hơn 0.3m
        if self.status in ["moving", "preparing"] and self.target_x is not None and self.target_y is not None:
            if self.pose_x is not None and self.pose_y is not None:
                dist = math.hypot(self.pose_x - self.target_x, self.pose_y - self.target_y)
                if dist < 0.3:
                    self.set_status("arrived")
                    self.status_hold_until = time.time() + 3.0

    def goal_pose_cb(self, msg: PoseStamped):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        if not self.current_goal or self.current_goal.startswith("Goal("):
            x = round(self.target_x, 1)
            y = round(self.target_y, 1)
            self.current_goal = f"Goal({x}, {y})"

        self.set_status("preparing")

        # Gửi Action đến Nav2
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("NavigateToPose action server not available!")
            self.set_status("error")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f"Sending goal to Nav2: x={self.target_x}, y={self.target_y}")
        self.nav_to_pose_client.send_goal_async(goal_msg)

    def goal_name_cb(self, msg: String):
        self.current_goal = msg.data.strip()

    def battery_cb(self, msg: BatteryState):
        self.battery_percent = float(msg.percentage)

    def battery_float_cb(self, msg: Float32):
        self.battery_percent = float(msg.data)

    def nav2_status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        latest_goal = msg.status_list[-1]
        nav_status = latest_goal.status

        if nav_status == GoalStatus.STATUS_ACCEPTED:
            self.set_status("preparing")
        elif nav_status == GoalStatus.STATUS_EXECUTING:
            self.set_status("moving")
        elif nav_status == GoalStatus.STATUS_CANCELING:
            self.set_status("canceling")
        elif nav_status == GoalStatus.STATUS_SUCCEEDED:
            if self.status != "arrived":
                self.set_status("arrived")
                self.status_hold_until = time.time() + 3.0
        elif nav_status == GoalStatus.STATUS_CANCELED:
            if self.status != "canceled":
                self.set_status("canceled")
                self.status_hold_until = time.time() + 3.0
        elif nav_status == GoalStatus.STATUS_ABORTED:
            if self.status != "error":
                self.set_status("error")
                self.status_hold_until = time.time() + 3.0

    def set_status(self, new_status: str):
        if self.status != new_status:
            self.status = new_status
            self.last_status_change_time = time.time()
            self.get_logger().info(f'Status changed to: {self.status}')

    def publish_status(self):
        now = time.time()

        if self.status_hold_until > 0 and now >= self.status_hold_until:
            self.status_hold_until = 0.0
            self.set_status("idle")
            self.current_goal = None
            self.target_x = None
            self.target_y = None

        pose_data = None
        if self.pose_x is not None and self.pose_y is not None:
            pose_data = {
                "x": round(self.pose_x, 2),
                "y": round(self.pose_y, 2)
            }

        payload = {
            "status": self.status,
            "current_goal": self.current_goal if self.status != "idle" else None,
            "battery_percent": round(self.battery_percent, 1) if self.battery_percent is not None else None,
            "pose": pose_data
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher_.publish(msg)


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
