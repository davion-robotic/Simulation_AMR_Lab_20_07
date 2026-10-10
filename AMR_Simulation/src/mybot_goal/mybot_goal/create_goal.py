#!/usr/bin/env python3
"""create_goal.py: Node ROS 2 xử lý việc tạo Goal mới (hỗ trợ tự động bắt tọa độ hiện tại của robot)."""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException

from mybot_common.topic_names import (
    GOAL_CREATE_TOPIC,
    GOAL_RESULT_TOPIC,
    GOAL_LIST_TOPIC,
    GOAL_AVAILABLE_GOALS_TOPIC,
    AMCL_POSE_TOPIC,
    ODOM_TOPIC,
)
from mybot_common import parse_payload_dict
from mybot_goal.goal_storage import GoalStorage


class CreateGoalNode(Node):
    """ROS 2 Node chuyên trách việc tạo Goal."""

    def __init__(self):
        super().__init__('create_goal_node')
        self.storage = GoalStorage()
        self.latest_amcl_pose = None
        self.latest_odom_pose = None

        # TF Listener for Nav2 frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Goal Topic Subscribers & Publishers
        self.subscription = self.create_subscription(
            String, GOAL_CREATE_TOPIC, self.on_create_goal, 10)

        self.pub_result = self.create_publisher(String, GOAL_RESULT_TOPIC, 10)
        self.pub_list = self.create_publisher(String, GOAL_LIST_TOPIC, 10)
        self.pub_available_goals = self.create_publisher(String, GOAL_AVAILABLE_GOALS_TOPIC, 10)

        # Robot Pose Tracking Subscribers
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self.amcl_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_cb, 10)

        self.get_logger().info('CreateGoalNode ready. Tracking Nav2 TF (map->base_footprint), /amcl_pose & /odom for goal capture.')

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = self._quaternion_to_yaw(q)
        self.latest_amcl_pose = {"x": round(p.x, 3), "y": round(p.y, 3), "theta": round(theta, 3)}

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = self._quaternion_to_yaw(q)
        self.latest_odom_pose = {"x": round(p.x, 3), "y": round(p.y, 3), "theta": round(theta, 3)}

    def get_current_pose(self) -> dict:
        """Return latest robot pose (Nav2 TF map->base_footprint preferred, fallback to AMCL, then Odom)."""
        # 1. Try Nav2 TF lookup (map -> base_footprint or base_link)
        for target_frame in ['base_footprint', 'base_link']:
            try:
                if self.tf_buffer.can_transform('map', target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)):
                    trans = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
                    p = trans.transform.translation
                    q = trans.transform.rotation
                    theta = self._quaternion_to_yaw(q)
                    self.get_logger().info(f"Captured robot pose via Nav2 TF (map -> {target_frame})")
                    return {"x": round(p.x, 3), "y": round(p.y, 3), "theta": round(theta, 3)}
            except TransformException as e:
                self.get_logger().debug(f"TF lookup failed for frame {target_frame}: {e}")

        # 2. Fallback to /amcl_pose topic
        if self.latest_amcl_pose is not None:
            self.get_logger().info("Captured robot pose via /amcl_pose topic")
            return self.latest_amcl_pose

        # 3. Fallback to /odom topic
        if self.latest_odom_pose is not None:
            self.get_logger().info("Captured robot pose via /odom topic")
            return self.latest_odom_pose

        return None

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def on_create_goal(self, msg: String):
        payload = parse_payload_dict(msg.data)
        map_name = payload.get("map_name", "")
        if not map_name:
            self.publish_result({"status": "error", "action": "create", "message": "Missing map_name"})
            return

        # Tự động bắt vị trí hiện tại nếu UI không gửi 'pose' hoặc đặt 'use_current_pose': true
        use_current = payload.get("use_current_pose", False) or "pose" not in payload or not payload.get("pose")
        if use_current:
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().error("Cannot create goal with current pose: No pose received from Nav2 TF, /amcl_pose or /odom yet")
                self.publish_result({
                    "status": "error",
                    "action": "create",
                    "message": "No robot pose available from Nav2 TF, /amcl_pose or /odom. Ensure robot is localized or TF/odom is active."
                })
                return
            payload["pose"] = current_pose.copy()
            self.get_logger().info(f"Captured current robot pose: {payload['pose']}")
        else:
            # Chuẩn hóa và làm tròn dữ liệu pose nếu UI tự gửi tọa độ tùy chỉnh
            pose = payload.get("pose", {})
            if isinstance(pose, dict):
                payload["pose"] = {
                    "x": round(float(pose.get("x", 0.0)), 3),
                    "y": round(float(pose.get("y", 0.0)), 3),
                    "theta": round(float(pose.get("theta", 0.0)), 3)
                }

        try:
            created = self.storage.create_goal(map_name, payload)
            self.get_logger().info(f"Created goal '{created['goal_id']}' for map '{map_name}' at pose {created['pose']}")
            self.publish_result({"status": "ok", "action": "create", "goal": created})
            self.publish_goals_list(map_name)
        except Exception as e:
            self.get_logger().error(f"Error creating goal: {e}")
            self.publish_result({"status": "error", "action": "create", "message": str(e)})

    def publish_goals_list(self, map_name: str):
        goals = self.storage.load_goals(map_name)
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

        payload = {"status": "ok", "map_name": map_name, "goals": goals}
        out = String()
        out.data = json.dumps(payload)
        self.pub_list.publish(out)

    def publish_result(self, result_dict: dict):
        out = String()
        out.data = json.dumps(result_dict)
        self.pub_result.publish(out)


def main():
    rclpy.init()
    node = CreateGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
