#!/usr/bin/env python3
"""initial_pose.py: Node ROS 2 xử lý việc đặt vị trí ước lượng ban đầu (Initial Pose) cho AMCL.

Hỗ trợ:
1. Đặt tọa độ thủ công qua JSON {"pose": {"x": 1.5, "y": 2.0, "theta": 0.0}}
2. Tự động lưu vị trí mới nhất của robot vào file last_known_pose.json qua topic /amcl_pose
3. Tự động nạp vị trí lưu gần nhất khi phát lệnh {"use_last_pose": true}
"""

import json
import math
import os
import subprocess
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException

from mybot_common.constants import MAPS_DIR
from mybot_common.command_utils import parse_payload_dict
from mybot_common.topic_names import (
    LOCALIZATION_INITIAL_POSE_TOPIC,
    LOCALIZATION_STATUS_TOPIC,
    NAV2_INITIAL_POSE_TOPIC,
    AMCL_POSE_TOPIC,
    ODOM_TOPIC,
)

LAST_KNOWN_POSE_FILE = MAPS_DIR / "last_known_pose.json"


def _parse_payload(data_str: str) -> dict:
    if not data_str:
        return {}
    try:
        parsed = json.loads(data_str)
        if isinstance(parsed, dict):
            if "data" in parsed and isinstance(parsed["data"], dict):
                return parsed["data"]
            return parsed
        return {}
    except Exception:
        return {}


class InitialPoseNode(Node):
    """ROS 2 Node chuyên trách việc đặt vị trí ban đầu Initial Pose cho AMCL."""

    def __init__(self):
        super().__init__('initial_pose_node')
        os.makedirs(MAPS_DIR, exist_ok=True)

        self.latest_amcl_pose = None
        self.latest_odom_pose = None

        # TF Listener for Nav2 frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Command Subscriber & Publisher
        self.subscription = self.create_subscription(
            String, LOCALIZATION_INITIAL_POSE_TOPIC, self.on_initial_pose_command, 10)

        self.pub_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, NAV2_INITIAL_POSE_TOPIC, 10)

        self.status_publisher = self.create_publisher(
            String, LOCALIZATION_STATUS_TOPIC, 10)

        # Pose Tracking Subscribers for Auto-Saving Last Pose
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self.amcl_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_cb, 10)

        self.get_logger().info(f'InitialPoseNode ready. Tracking TF/amcl/odom pose & managing {LAST_KNOWN_POSE_FILE}')

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = self._quaternion_to_yaw(q)
        self.latest_amcl_pose = {"x": round(p.x, 3), "y": round(p.y, 3), "theta": round(theta, 3)}
        self.save_last_known_pose(self.latest_amcl_pose)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = self._quaternion_to_yaw(q)
        self.latest_odom_pose = {"x": round(p.x, 3), "y": round(p.y, 3), "theta": round(theta, 3)}
        if self.latest_amcl_pose is None:
            self.save_last_known_pose(self.latest_odom_pose)

    def get_current_pose(self) -> dict:
        """Return latest robot pose (Nav2 TF map->base_footprint preferred, fallback to AMCL, then Odom)."""
        for target_frame in ['base_footprint', 'base_link']:
            try:
                if self.tf_buffer.can_transform('map', target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)):
                    trans = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
                    p = trans.transform.translation
                    q = trans.transform.rotation
                    theta = self._quaternion_to_yaw(q)
                    return {"x": round(p.x, 3), "y": round(p.y, 3), "theta": round(theta, 3)}
            except TransformException:
                pass

        if self.latest_amcl_pose is not None:
            return self.latest_amcl_pose

        return self.latest_odom_pose

    def save_last_known_pose(self, pose: dict):
        """Auto-save latest valid pose to file."""
        try:
            with open(LAST_KNOWN_POSE_FILE, 'w', encoding='utf-8') as f:
                json.dump({"pose": pose, "updated_at": self.get_clock().now().to_msg().sec}, f, indent=2)
        except Exception:
            pass

    def load_last_known_pose(self) -> dict:
        """Load last known pose from file."""
        if not LAST_KNOWN_POSE_FILE.is_file():
            return None
        try:
            with open(LAST_KNOWN_POSE_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if isinstance(data, dict) and "pose" in data:
                    return data["pose"]
        except Exception:
            pass
        return None

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def on_initial_pose_command(self, msg: String):
        payload = parse_payload_dict(msg.data)
        use_last_pose = payload.get("use_last_pose", False)
        use_current_pose = payload.get("use_current_pose", False)

        x, y, theta = 0.0, 0.0, 0.0
        pose_source = "explicit"

        if use_current_pose:
            curr_pose = self.get_current_pose()
            if curr_pose is not None:
                x = float(curr_pose.get("x", 0.0))
                y = float(curr_pose.get("y", 0.0))
                theta = float(curr_pose.get("theta", 0.0))
                pose_source = "current_pose"
                self.get_logger().info(f'Captured current robot pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
            else:
                self.get_logger().error('use_current_pose requested but no pose received from TF/AMCL/Odom yet!')
                self.publish_status({"status": "error", "message": "No robot pose available yet"})
                return
        elif use_last_pose or ("pose" not in payload and "x" not in payload):
            last_pose = self.load_last_known_pose()
            if last_pose is not None:
                x = float(last_pose.get("x", 0.0))
                y = float(last_pose.get("y", 0.0))
                theta = float(last_pose.get("theta", 0.0))
                pose_source = "last_known_pose"
                self.get_logger().info(f'Loaded last known pose from file: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
            else:
                self.get_logger().warning('No last known pose file found. Falling back to default (0, 0, 0)')
        else:
            pose_data = payload.get("pose", payload)
            x = float(pose_data.get("x", 0.0))
            y = float(pose_data.get("y", 0.0))
            theta = float(pose_data.get("theta", 0.0))

        # Build PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion (yaw about Z-axis)
        pose_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Standard initial covariance matrix
        cov = [0.0] * 36
        cov[0] = 0.25   # x
        cov[7] = 0.25   # y
        cov[35] = 0.068 # yaw
        pose_msg.pose.covariance = cov

        self.pub_initial_pose.publish(pose_msg)
        self.save_last_known_pose({"x": x, "y": y, "theta": theta})
        self.get_logger().info(f'Published initial pose ({pose_source}): x={x:.2f}, y={y:.2f}, theta={theta:.2f} rad to {NAV2_INITIAL_POSE_TOPIC}')



        self.publish_status({
            "status": "ok",
            "action": "initial_pose",
            "source": pose_source,
            "pose": {"x": x, "y": y, "theta": theta}
        })



    def publish_status(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.status_publisher.publish(msg)


def main():
    rclpy.init()
    node = InitialPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
