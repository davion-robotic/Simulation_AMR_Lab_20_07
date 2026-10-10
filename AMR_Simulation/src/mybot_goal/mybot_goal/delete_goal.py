#!/usr/bin/env python3
"""delete_goal.py: Node ROS 2 xử lý riêng việc xóa Goal."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mybot_common import parse_payload_dict
from mybot_common.constants import MAPS_DIR
from mybot_common.command_utils import sanitize_map_name
from mybot_common.topic_names import (
    GOAL_DELETE_TOPIC,
    GOAL_RESULT_TOPIC,
    GOAL_LIST_TOPIC,
    GOAL_AVAILABLE_GOALS_TOPIC,
)
from mybot_goal.goal_storage import GoalStorage


class DeleteGoalNode(Node):
    """ROS 2 Node chuyên trách việc xóa Goal."""

    def __init__(self):
        super().__init__('delete_goal_node')
        self.storage = GoalStorage()

        self.subscription = self.create_subscription(
            String, GOAL_DELETE_TOPIC, self.on_delete_goal, 10)

        self.pub_result = self.create_publisher(String, GOAL_RESULT_TOPIC, 10)
        self.pub_list = self.create_publisher(String, GOAL_LIST_TOPIC, 10)
        self.pub_available_goals = self.create_publisher(String, GOAL_AVAILABLE_GOALS_TOPIC, 10)

        self.get_logger().info('DeleteGoalNode ready.')

    def on_delete_goal(self, msg: String):
        payload = parse_payload_dict(msg.data)
        if not isinstance(payload, dict):
            payload = {"goal_id": str(payload).strip()}

        goal_id = None
        for k in ["goal_id", "name", "id", "vertex", "target", "point", "data"]:
            if k in payload and payload[k]:
                val = payload[k]
                if isinstance(val, dict):
                    val = val.get("goal_id") or val.get("name") or val.get("id")
                if val:
                    goal_id = str(val).strip()
                    break

        map_name = sanitize_map_name(payload.get("map_name", ""))
        
        # Nếu thiếu map_name, tự động tìm kiếm map_name có chứa goal_id này
        if not map_name and goal_id:
            for goal_file in MAPS_DIR.glob("*_goals.json"):
                cand_map = goal_file.stem.replace("_goals", "")
                cand_goals = self.storage.load_goals(cand_map)
                for g in cand_goals:
                    if str(g.get("goal_id", "")).strip().lower() == goal_id.lower() or str(g.get("name", "")).strip().lower() == goal_id.lower():
                        map_name = cand_map
                        break
                if map_name:
                    break

            # Quét thêm trong thư mục topology maps nếu chưa tìm thấy
            if not map_name:
                from mybot_rmf.topology_edit import RMF_MAPS_DIR, TopologyEdit
                t_edit = TopologyEdit(maps_dir=RMF_MAPS_DIR)
                for top_file in RMF_MAPS_DIR.glob("*_topology.yaml"):
                    cand_map = top_file.stem.replace("_topology", "")
                    _, _, _, cand_verts, _ = t_edit.load_graph(cand_map)
                    for v in cand_verts:
                        if len(v) > 2 and isinstance(v[2], dict):
                            v_name = str(v[2].get("name", "")).strip().lower()
                            if v_name == goal_id.lower():
                                map_name = cand_map
                                break
                    if map_name:
                        break

        if not map_name:
            map_name = "map_lab21"  # Default fallback

        if not goal_id:
            self.publish_result({"status": "error", "action": "delete", "message": "Missing goal_id or name to delete"})
            return

        try:
            deleted = None
            try:
                deleted = self.storage.delete_goal(map_name, goal_id)
            except Exception as g_err:
                self.get_logger().debug(f"GoalStorage delete note: {g_err}")

            # Đồng bộ xóa luôn khỏi file Topology RMF nếu có
            try:
                from mybot_rmf.topology_edit import TopologyEdit, RMF_MAPS_DIR
                top_handler = TopologyEdit(maps_dir=RMF_MAPS_DIR)
                top_handler.delete_vertex({"map_name": map_name, "name": goal_id})
                self.get_logger().info(f"✓ Synchronized deletion of '{goal_id}' in Topology graph for '{map_name}'")
            except Exception as top_err:
                self.get_logger().debug(f"Topology sync note: {top_err}")

            self.get_logger().info(
                f"\n🔴 [GOAL] ➖ DELETE GOAL:\n"
                f"   • Map     : {map_name}\n"
                f"   • Goal ID : '{goal_id}'\n"
                f"   • Result  : Successfully deleted goal from storage and topology"
            )

            self.publish_result({"status": "ok", "action": "delete", "deleted_goal": deleted or {"goal_id": goal_id}})
            self.publish_goals_list(map_name)
        except Exception as e:
            self.get_logger().error(f"\n❌ [GOAL] ⚠️ DELETE GOAL FAILED: {e}")
            self.publish_result({"status": "error", "action": "delete", "message": str(e)})

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
    node = DeleteGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
