#!/usr/bin/env python3
"""reset_pose.py: Node ROS 2 xử lý việc reset vị trí định vị robot về mốc (0, 0, 0)."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from mybot_common.topic_names import (
    LOCALIZATION_RESET_POSE_TOPIC,
    LOCALIZATION_STATUS_TOPIC,
    NAV2_INITIAL_POSE_TOPIC,
)


class ResetPoseNode(Node):
    """ROS 2 Node chuyên trách việc reset vị trí AMCL của robot."""

    def __init__(self):
        super().__init__('reset_pose_node')

        self.subscription = self.create_subscription(
            String, LOCALIZATION_RESET_POSE_TOPIC, self.on_reset_command, 10)

        self.pub_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, NAV2_INITIAL_POSE_TOPIC, 10)

        self.status_publisher = self.create_publisher(
            String, LOCALIZATION_STATUS_TOPIC, 10)

        self.get_logger().info('ResetPoseNode ready. Subscribing to /localization/reset_pose')

    def on_reset_command(self, msg: String):
        # Build zero pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0

        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        cov[0] = 0.5   # x
        cov[7] = 0.5   # y
        cov[35] = 0.1  # yaw
        pose_msg.pose.covariance = cov

        self.pub_initial_pose.publish(pose_msg)
        self.get_logger().info('Reset robot initial pose to (0, 0, 0) origin.')

        self.publish_status({
            "status": "ok",
            "action": "reset_pose",
            "pose": {"x": 0.0, "y": 0.0, "theta": 0.0}
        })

    def publish_status(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.status_publisher.publish(msg)


def main():
    rclpy.init()
    node = ResetPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
