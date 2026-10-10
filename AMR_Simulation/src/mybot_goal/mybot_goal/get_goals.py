#!/usr/bin/env python3
"""get_goals.py: Node ROS 2 xử lý riêng việc lấy danh sách Goal."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mybot_common.topic_names import (
    GOAL_GET_LIST_REQ_TOPIC,
    GOAL_LIST_TOPIC,
    GOAL_AVAILABLE_GOALS_TOPIC,
    GOAL_RESULT_TOPIC,
)
from mybot_goal.goal_storage import GoalStorage


def _parse_payload(data_str: str) -> dict:
    if not data_str:
        return {}
    try:
        parsed = json.loads(data_str)
        if isinstance(parsed, dict):
            if "data" in parsed and isinstance(parsed["data"], dict):
                return parsed["data"]
            return parsed
        return {"map_name": str(parsed)}
    except Exception:
        return {"map_name": data_str.strip()}


class GetGoalsNode(Node):
    """ROS 2 Node chuyên trách việc lấy/quét danh sách Goal."""

    def __init__(self):
        super().__init__('get_goals_node')
        self.storage = GoalStorage()

        self.subscription = self.create_subscription(
            String, GOAL_GET_LIST_REQ_TOPIC, self.on_get_goals_req, 10)

        self.pub_list = self.create_publisher(String, GOAL_LIST_TOPIC, 10)
        self.pub_available_goals = self.create_publisher(String, GOAL_AVAILABLE_GOALS_TOPIC, 10)
        self.pub_result = self.create_publisher(String, GOAL_RESULT_TOPIC, 10)

        self.get_logger().info('GetGoalsNode ready.')

    def on_get_goals_req(self, msg: String):
        payload = _parse_payload(msg.data)
        map_name = payload.get("map_name", "")
        if not map_name:
            self.publish_result({"status": "error", "action": "list", "message": "Missing map_name"})
            return

        goals = self.storage.load_goals(map_name)
        self.get_logger().info(f"Found {len(goals)} goals for map '{map_name}'")

        # 1. Format JSON Array phẳng cho UI (/goal_manager/available_goals)
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

        # 2. Format dict chuẩn cho hệ thống nội bộ (/goal_manager/goals_list)
        out_list = String()
        out_list.data = json.dumps({"status": "ok", "map_name": map_name, "goals": goals})
        self.pub_list.publish(out_list)

    def publish_result(self, result_dict: dict):
        out = String()
        out.data = json.dumps(result_dict)
        self.pub_result.publish(out)


def main():
    rclpy.init()
    node = GetGoalsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
