#!/usr/bin/env python3
"""GoalManagerNode: ROS 2 node to handle Goal creation, updating, deletion, and listing."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mybot_common.topic_names import (
    GOAL_CREATE_TOPIC,
    GOAL_UPDATE_TOPIC,
    GOAL_DELETE_TOPIC,
    GOAL_GET_LIST_REQ_TOPIC,
    GOAL_LIST_TOPIC,
    GOAL_AVAILABLE_GOALS_TOPIC,
    GOAL_RESULT_TOPIC,
)
from mybot_common.status_utils import format_status_json, create_status_payload
from mybot_goal.goal_storage import GoalStorage


def _parse_payload(data_str: str) -> dict:
    """Parse JSON string into dict robustly."""
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


class GoalManagerNode(Node):
    """ROS 2 Node for managing waypoints/goals."""

    def __init__(self):
        super().__init__('goal_manager')

        self.storage = GoalStorage()

        # Create Goal Subscriber
        self.sub_create = self.create_subscription(
            String, GOAL_CREATE_TOPIC, self.on_create_goal, 10)

        # Update Goal Subscriber
        self.sub_update = self.create_subscription(
            String, GOAL_UPDATE_TOPIC, self.on_update_goal, 10)

        # Delete Goal Subscriber
        self.sub_delete = self.create_subscription(
            String, GOAL_DELETE_TOPIC, self.on_delete_goal, 10)

        # Get List Request Subscriber
        self.sub_list_req = self.create_subscription(
            String, GOAL_GET_LIST_REQ_TOPIC, self.on_get_goals_req, 10)

        # Publishers
        self.pub_result = self.create_publisher(String, GOAL_RESULT_TOPIC, 10)
        self.pub_list = self.create_publisher(String, GOAL_LIST_TOPIC, 10)
        self.pub_available_goals = self.create_publisher(String, GOAL_AVAILABLE_GOALS_TOPIC, 10)

        self.get_logger().info('GoalManagerNode ready.')

    def on_create_goal(self, msg: String):
        payload = _parse_payload(msg.data)
        map_name = payload.get("map_name", "")
        if not map_name:
            self.publish_result({"status": "error", "action": "create", "message": "Missing map_name"})
            return

        try:
            created = self.storage.create_goal(map_name, payload)
            self.get_logger().info(f"Created goal '{created['goal_id']}' for map '{map_name}'")
            self.publish_result({"status": "ok", "action": "create", "goal": created})
            # Also publish updated list
            self.publish_goals_list(map_name)
        except Exception as e:
            self.get_logger().error(f"Error creating goal: {e}")
            self.publish_result({"status": "error", "action": "create", "message": str(e)})

    def on_update_goal(self, msg: String):
        payload = _parse_payload(msg.data)
        map_name = payload.get("map_name", "")
        if not map_name:
            self.publish_result({"status": "error", "action": "update", "message": "Missing map_name"})
            return

        try:
            updated = self.storage.update_goal(map_name, payload)
            self.get_logger().info(f"Updated goal '{updated['goal_id']}' for map '{map_name}'")
            self.publish_result({"status": "ok", "action": "update", "goal": updated})
            # Also publish updated list
            self.publish_goals_list(map_name)
        except Exception as e:
            self.get_logger().error(f"Error updating goal: {e}")
            self.publish_result({"status": "error", "action": "update", "message": str(e)})

    def on_delete_goal(self, msg: String):
        payload = _parse_payload(msg.data)
        map_name = payload.get("map_name", "")
        goal_id = payload.get("goal_id") or payload.get("name")
        if not map_name or not goal_id:
            self.publish_result({"status": "error", "action": "delete", "message": "Missing map_name or goal_id"})
            return

        try:
            deleted = self.storage.delete_goal(map_name, goal_id)
            self.get_logger().info(f"Deleted goal '{goal_id}' from map '{map_name}'")
            self.publish_result({"status": "ok", "action": "delete", "deleted_goal": deleted})
            # Also publish updated list
            self.publish_goals_list(map_name)
        except Exception as e:
            self.get_logger().error(f"Error deleting goal: {e}")
            self.publish_result({"status": "error", "action": "delete", "message": str(e)})

    def on_get_goals_req(self, msg: String):
        payload = _parse_payload(msg.data)
        map_name = payload.get("map_name", "")
        if not map_name:
            self.publish_result({"status": "error", "action": "list", "message": "Missing map_name"})
            return
        self.publish_goals_list(map_name)

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
    node = GoalManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
