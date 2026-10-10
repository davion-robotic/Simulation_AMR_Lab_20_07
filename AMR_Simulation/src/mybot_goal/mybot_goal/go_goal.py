#!/usr/bin/env python3
"""go_goal.py: Node ROS 2 xử lý việc cho Robot di chuyển đến Goal chỉ định trên Nav2 (Hỗ trợ bám theo đường Topology)."""

import json
import math
import os
import yaml
import collections
from pathlib import Path
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

from mybot_common.topic_names import (
    GOAL_GO_TO_TOPIC,
    GOAL_CANCEL_TOPIC,
    GOAL_RESULT_TOPIC,
)
from mybot_common import parse_payload_dict
from mybot_goal.goal_storage import GoalStorage


def project_point_on_segment(cx, cy, ax, ay, bx, by):
    """Tính điểm chiếu vuông góc của C(cx, cy) lên đoạn thẳng AB."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return ax, ay, math.hypot(cx - ax, cy - ay), 0.0
    t = ((cx - ax) * dx + (cy - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    px = ax + t * dx
    py = ay + t * dy
    dist = math.hypot(cx - px, cy - py)
    return px, py, dist, t


class GoGoalNode(Node):
    """ROS 2 Node chuyên trách ra lệnh cho Robot di chuyển tới vị trí Goal đã lưu hoặc tọa độ chỉ định.
    Hỗ trợ tự động bám theo các làn đường Topology Graph (nếu có) hoặc fallback về NavigateToPose.
    """

    def __init__(self):
        super().__init__('go_goal_node')
        self.storage = GoalStorage()

        # Action clients kết nối với Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # Vị trí hiện tại của Robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Subscriptions & TF Listener để theo dõi vị trí thực tế của robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Topic Subscribers
        self.sub_go_to = self.create_subscription(
            String, GOAL_GO_TO_TOPIC, self.on_go_to_goal, 10)

        self.sub_cancel = self.create_subscription(
            String, GOAL_CANCEL_TOPIC, self.on_cancel_goal, 10)

        # Topic Publisher
        self.pub_result = self.create_publisher(String, GOAL_RESULT_TOPIC, 10)

        self.current_goal_handle = None

        self.get_logger().info('GoGoalNode ready with Topology support. Listening on /goal_manager/go_to_goal & /goal_manager/cancel_goal.')

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = float(p.x)
        self.robot_y = float(p.y)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def update_robot_pose_from_tf(self):
        try:
            if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)):
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                p = t.transform.translation
                q = t.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.robot_x = float(p.x)
                self.robot_y = float(p.y)
                self.robot_yaw = math.atan2(siny, cosy)
        except Exception:
            pass

    def load_topology_graph(self, map_name: str = "map_lab20"):
        """Đọc đồ thị Topology <map_name>_topology.yaml."""
        target_map = map_name or "map_lab20"
        candidates = [
            f'/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_rmf/maps/{target_map}_topology.yaml',
            f'/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_rmf/maps/{target_map}topology.yaml',
            '/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_rmf/maps/map_lab20_topology.yaml'
        ]
        try:
            pkg_share = get_package_share_directory('mybot_rmf')
            candidates.insert(0, os.path.join(pkg_share, 'maps', f'{target_map}_topology.yaml'))
        except Exception:
            pass

        graph_path = None
        for c in candidates:
            if os.path.exists(c):
                graph_path = c
                break

        if not graph_path:
            return [], []

        try:
            with open(graph_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            levels = data.get('levels', {})
            for l_name, l_data in levels.items():
                if 'vertices' in l_data and 'lanes' in l_data:
                    return l_data['vertices'], l_data['lanes']
        except Exception as e:
            self.get_logger().error(f"Error loading topology graph '{graph_path}': {e}")

        return [], []

    def find_closest_vertex_idx(self, x: float, y: float, vertices: list) -> int:
        best_idx = 0
        min_dist = float('inf')
        for i, v in enumerate(vertices):
            vx, vy = float(v[0]), float(v[1])
            dist = math.hypot(x - vx, y - vy)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        return best_idx

    def find_closest_lane_projection(self, cx: float, cy: float, vertices: list, lanes: list):
        best_proj = (cx, cy)
        best_dist = float('inf')
        best_lane = None
        best_t = 0.0

        for lane in lanes:
            u_idx, v_idx = lane[0], lane[1]
            if u_idx < len(vertices) and v_idx < len(vertices):
                ax, ay = float(vertices[u_idx][0]), float(vertices[u_idx][1])
                bx, by = float(vertices[v_idx][0]), float(vertices[v_idx][1])
                px, py, dist, t = project_point_on_segment(cx, cy, ax, ay, bx, by)
                if dist < best_dist:
                    best_dist = dist
                    best_proj = (px, py)
                    best_lane = (u_idx, v_idx)
                    best_t = t

        return best_proj, best_dist, best_lane, best_t

    def find_shortest_topology_path(self, start_idx: int, target_idx: int, vertices: list, lanes: list) -> list:
        if start_idx == target_idx:
            return [start_idx]

        adj = collections.defaultdict(list)
        for lane in lanes:
            u, v = lane[0], lane[1]
            adj[u].append(v)
            opts = lane[2] if len(lane) > 2 else {}
            if opts.get('is_bidirectional', True):
                adj[v].append(u)

        queue = collections.deque([[start_idx]])
        visited = {start_idx}

        while queue:
            path = queue.popleft()
            curr = path[-1]
            if curr == target_idx:
                return path
            for neighbor in adj[curr]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(path + [neighbor])

        return [start_idx, target_idx]

    def generate_interpolated_poses(self, points: list, step_distance: float = 0.25, final_theta: float = 0.0) -> list:
        poses = []
        now = self.get_clock().now().to_msg()

        if len(points) == 1:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = now
            pose.pose.position.x = points[0][0]
            pose.pose.position.y = points[0][1]
            pose.pose.orientation.z = math.sin(final_theta / 2.0)
            pose.pose.orientation.w = math.cos(final_theta / 2.0)
            return [pose]

        for i in range(len(points) - 1):
            x1, y1 = points[i][0], points[i][1]
            x2, y2 = points[i+1][0], points[i+1][1]

            dist = math.hypot(x2 - x1, y2 - y1)
            if dist < 0.02:
                continue

            yaw = math.atan2(y2 - y1, x2 - x1)
            num_steps = max(1, int(math.ceil(dist / step_distance)))

            for s in range(1, num_steps + 1):
                fraction = float(s) / float(num_steps)
                px = x1 + fraction * (x2 - x1)
                py = y1 + fraction * (y2 - y1)

                is_last_point = (i == len(points) - 2) and (s == num_steps)
                current_yaw = final_theta if is_last_point else yaw

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = now
                pose.pose.position.x = px
                pose.pose.position.y = py
                pose.pose.orientation.z = math.sin(current_yaw / 2.0)
                pose.pose.orientation.w = math.cos(current_yaw / 2.0)

                poses.append(pose)

        return poses

    def publish_result(self, result_dict: dict):
        """Helper function to publish result JSON to GOAL_RESULT_TOPIC."""
        try:
            out = String()
            out.data = json.dumps(result_dict)
            self.pub_result.publish(out)
        except Exception as e:
            self.get_logger().error(f"Error publishing result: {e}")

    def on_go_to_goal(self, msg: String):
        payload = parse_payload_dict(msg.data)
        map_name = payload.get("map_name", "")
        goal_id = payload.get("goal_id", "")
        name = payload.get("name", "")
        target_pose = payload.get("pose", None)

        found_goal = None

        # 1. Nếu có chỉ định x, y trong payload -> Dùng pose trực tiếp
        if target_pose and isinstance(target_pose, dict) and "x" in target_pose and "y" in target_pose:
            found_goal = {
                "goal_id": goal_id or "CUSTOM",
                "name": name or "Custom Location",
                "pose": target_pose
            }
        else:
            # 2. Ngược lại -> Tìm Goal trong file JSON của map
            if not map_name:
                self.publish_result({
                    "status": "error",
                    "action": "go_to_goal",
                    "message": "Missing map_name or pose to navigate"
                })
                return

            if not goal_id and not name:
                self.publish_result({
                    "status": "error",
                    "action": "go_to_goal",
                    "message": "Missing goal_id or name to search"
                })
                return

            goals = self.storage.load_goals(map_name)
            search_key = str(goal_id or name).strip()

            for g in goals:
                if str(g.get("goal_id", "")).strip() == search_key or str(g.get("name", "")).strip() == search_key:
                    found_goal = g
                    break

        if not found_goal:
            self.get_logger().error(f"Goal '{goal_id or name}' not found in map '{map_name}'")
            self.publish_result({
                "status": "error",
                "action": "go_to_goal",
                "message": f"Goal '{goal_id or name}' not found for map '{map_name}'"
            })
            return

        pose_data = found_goal.get("pose", {})
        x = float(pose_data.get("x", 0.0))
        y = float(pose_data.get("y", 0.0))
        theta = float(pose_data.get("theta", 0.0))

        self.get_logger().info(f"Navigating to goal '{found_goal.get('name')}' (ID: {found_goal.get('goal_id')}) at x={x}, y={y}, theta={theta}")

        # Hủy goal cũ nếu đang có goal chưa hoàn tất
        if self.current_goal_handle is not None:
            self.get_logger().info("Canceling active navigation goal before sending new goal...")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        # 3. Thử tìm lộ trình Topology Graph
        vertices, lanes = self.load_topology_graph(map_name)
        route_pts = []

        if vertices and lanes:
            self.update_robot_pose_from_tf()
            start_idx = self.find_closest_vertex_idx(self.robot_x, self.robot_y, vertices)

            # Tìm xem goal có nằm gần vertex nào không
            target_vertex_idx = None
            search_name = str(found_goal.get("name", "")).strip()
            search_id = str(found_goal.get("goal_id", "")).strip()

            for i, v in enumerate(vertices):
                v_name = v[2].get('name', '') if len(v) > 2 else ''
                if v_name and (v_name == search_name or v_name == search_id):
                    target_vertex_idx = i
                    break

            if target_vertex_idx is None:
                # Kiểm tra khoảng cách tới vertex gần nhất (bán kính 0.4m)
                closest_v_idx = self.find_closest_vertex_idx(x, y, vertices)
                closest_v = vertices[closest_v_idx]
                if math.hypot(x - float(closest_v[0]), y - float(closest_v[1])) < 0.4:
                    target_vertex_idx = closest_v_idx

            if target_vertex_idx is not None:
                path_indices = self.find_shortest_topology_path(start_idx, target_vertex_idx, vertices, lanes)
                route_pts = [(float(vertices[idx][0]), float(vertices[idx][1])) for idx in path_indices]
            else:
                # Điểm nằm lân cận làn đường
                best_proj, proj_dist, best_lane, best_t = self.find_closest_lane_projection(x, y, vertices, lanes)
                if best_lane is not None:
                    u_idx, v_idx = best_lane
                    target_node_idx = u_idx if best_t <= 0.5 else v_idx
                    path_indices = self.find_shortest_topology_path(start_idx, target_node_idx, vertices, lanes)
                    route_pts = [(float(vertices[idx][0]), float(vertices[idx][1])) for idx in path_indices]
                    route_pts.append((best_proj[0], best_proj[1]))
                    if proj_dist > 0.05:
                        route_pts.append((x, y))

        # 4. Gửi Action
        if route_pts and self.nav_through_client.wait_for_server(timeout_sec=2.0):
            interpolated_poses = self.generate_interpolated_poses(route_pts, step_distance=0.25, final_theta=theta)
            if interpolated_poses:
                self.get_logger().info(f"⚡ Navigating via Topology route ({len(interpolated_poses)} waypoints) to '{found_goal.get('name')}'...")
                goal_msg = NavigateThroughPoses.Goal()
                goal_msg.poses = interpolated_poses

                send_goal_future = self.nav_through_client.send_goal_async(
                    goal_msg,
                    feedback_callback=self.feedback_cb
                )
                send_goal_future.add_done_callback(
                    partial(self.goal_response_cb, goal_info=found_goal)
                )

                self.publish_result({
                    "status": "executing",
                    "action": "go_to_goal",
                    "mode": "topology",
                    "goal_id": found_goal.get("goal_id"),
                    "name": found_goal.get("name"),
                    "pose": pose_data,
                    "message": f"Sending robot to '{found_goal.get('name')}' via Topology path"
                })
                return

        # Fallback: Dùng NavigateToPose trực tiếp nếu không tìm thấy Topology hoặc nav_through_client không sẵn sàng
        if not self.nav_to_pose_client.server_is_ready():
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Nav2 'navigate_to_pose' action server not available!")
                self.publish_result({
                    "status": "error",
                    "action": "go_to_goal",
                    "message": "Nav2 navigation action server is not active. Please launch navigation2.launch.py first."
                })
                return

        self.get_logger().info(f"📍 Fallback to Direct NavigateToPose for '{found_goal.get('name')}'...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        q_z = math.sin(theta / 2.0)
        q_w = math.cos(theta / 2.0)
        goal_msg.pose.pose.orientation.z = q_z
        goal_msg.pose.pose.orientation.w = q_w

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        send_goal_future.add_done_callback(
            partial(self.goal_response_cb, goal_info=found_goal)
        )

        self.publish_result({
            "status": "executing",
            "action": "go_to_goal",
            "mode": "direct",
            "goal_id": found_goal.get("goal_id"),
            "name": found_goal.get("name"),
            "pose": pose_data,
            "message": f"Sending robot directly to '{found_goal.get('name')}'"
        })

    def goal_response_cb(self, future, goal_info):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal '{goal_info.get('name')}' rejected by Nav2 action server")
            self.publish_result({
                "status": "rejected",
                "action": "go_to_goal",
                "goal": goal_info,
                "message": f"Navigation goal '{goal_info.get('name')}' rejected by Nav2"
            })
            return

        self.get_logger().info(f"Goal '{goal_info.get('name')}' accepted by Nav2, robot navigating...")
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self.get_result_cb, goal_handle=goal_handle, goal_info=goal_info)
        )

    def feedback_cb(self, feedback_msg):
        pass

    def get_result_cb(self, future, goal_handle, goal_info):
        result = future.result()
        status = result.status

        # Chỉ clear current_goal_handle khi chính handle này hoàn tất
        if self.current_goal_handle == goal_handle:
            self.current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Robot successfully reached goal '{goal_info.get('name')}'!")
            self.publish_result({
                "status": "ok",
                "action": "go_to_goal",
                "goal_id": goal_info.get("goal_id"),
                "name": goal_info.get("name"),
                "message": f"Robot reached goal '{goal_info.get('name')}' successfully."
            })
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Navigation to '{goal_info.get('name')}' was canceled.")
            self.publish_result({
                "status": "canceled",
                "action": "go_to_goal",
                "goal_id": goal_info.get("goal_id"),
                "name": goal_info.get("name"),
                "message": f"Navigation to '{goal_info.get('name')}' canceled."
            })
        else:
            self.get_logger().error(f"Navigation to '{goal_info.get('name')}' failed with status {status}")
            self.publish_result({
                "status": "error",
                "action": "go_to_goal",
                "goal_id": goal_info.get("goal_id"),
                "name": goal_info.get("name"),
                "message": f"Failed to reach goal '{goal_info.get('name')}' (status: {status})."
            })

    def on_cancel_goal(self, msg: String):
        if self.current_goal_handle is not None:
            self.get_logger().info("Canceling active navigation goal...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_cb)
        else:
            self.publish_result({
                "status": "ok",
                "action": "cancel_goal",
                "message": "No active navigation goal to cancel."
            })

    def cancel_done_cb(self, future):
        self.publish_result({
            "status": "ok",
            "action": "cancel_goal",
            "message": "Navigation goal cancellation requested."
        })


def main():
    rclpy.init()
    node = GoGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

