#!/usr/bin/env python3
"""
goto_waypoint.py (mybot_rmf)
----------------------------
Node điều hướng Robot bám 100% theo các nét vẽ Topology RMF (Lanes):
- Xử lý bất đồng bộ (Asynchronous Threading): Nhận liên tiếp nhiều điểm goal từ RViz2
  mà không bị treo hay nghẽn ROS 2 Executor!
- Hỗ trợ click trạm đỗ hoặc điểm bất kỳ lân cận làn (Lanes).
- Hủy Goal cũ tự động khi click Goal mới.
"""

import os
import sys
import yaml
import math
import collections
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory


def project_point_on_segment(cx, cy, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return ax, ay, math.hypot(cx - ax, cy - ay), 0.0
    t = ((cx - ax) * dx + (cy - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    px = ax + t * dx
    py = ay + t * dy
    dist = math.hypot(cx - px, cy - py)
    return px, py, dist, t


class GotoWaypoint(Node):
    def __init__(self):
        super().__init__('goto_waypoint')
        self.declare_parameter('waypoint', '')

        # Action Clients
        self._nav_through_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self._nav_to_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Vị trí hiện tại của Robot
        self.robot_x = 0.369
        self.robot_y = 2.58
        self.robot_yaw = 0.0

        # Quản lý Threading & Goal Handle
        self._current_goal_handle = None
        self._nav_thread = None
        self._lock = threading.Lock()

        # Subscriptions lấy vị trí robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Lắng nghe 2D Nav Goal từ RViz2
        self.sub_rmf_goal = self.create_subscription(
            PoseStamped, '/rmf_goal_pose', self.on_rviz_pose, 10)
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.on_rviz_pose, 10)

        self.get_logger().info("Topology Smart Async Navigator Ready! Click consecutive goals anytime on RViz2!")

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

    def load_graph(self, map_name: str = "map_lab20"):
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
            self.get_logger().error(f"Cannot find topology graph for map '{target_map}'")
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

    def generate_interpolated_poses(self, points: list, step_distance: float = 0.25) -> list:
        poses = []
        now = self.get_clock().now().to_msg()

        for i in range(len(points) - 1):
            x1, y1 = points[i][0], points[i][1]
            x2, y2 = points[i+1][0], points[i+1][1]

            dist = math.hypot(x2 - x1, y2 - y1)
            if dist < 0.05:
                continue

            yaw = math.atan2(y2 - y1, x2 - x1)
            num_steps = max(1, int(math.ceil(dist / step_distance)))

            for s in range(1, num_steps + 1):
                fraction = float(s) / float(num_steps)
                px = x1 + fraction * (x2 - x1)
                py = y1 + fraction * (y2 - y1)

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = now
                pose.pose.position.x = px
                pose.pose.position.y = py
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

                poses.append(pose)

        return poses

    def cancel_active_navigation(self):
        with self._lock:
            if self._current_goal_handle is not None:
                self.get_logger().info("Cancelling previous active navigation goal...")
                try:
                    self._current_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self._current_goal_handle = None

    def start_navigation_async(self, route_pts: list, label: str):
        # Tắt goal cũ nếu robot đang di chuyển
        self.cancel_active_navigation()

        # Chạy navigation trên background thread để không làm nghẽn ROS 2 Executor
        self._nav_thread = threading.Thread(
            target=self._navigation_worker,
            args=(route_pts, label),
            daemon=True
        )
        self._nav_thread.start()

    def _navigation_worker(self, route_pts: list, label: str):
        interpolated_poses = self.generate_interpolated_poses(route_pts, step_distance=0.25)
        if not interpolated_poses:
            return

        if not self._nav_through_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Action Server '/navigate_through_poses' is NOT available!")
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = interpolated_poses

        self.get_logger().info(f"Sending Topology route ({len(interpolated_poses)} waypoints) to Nav2 for '{label}'...")

        send_future = self._nav_through_client.send_goal_async(goal)
        
        # Chờ phản hồi trong worker thread
        while rclpy.ok() and not send_future.done():
            threading.Event().wait(0.05)

        if not send_future.done() or send_future.cancelled():
            return

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Nav2 rejected goal route for '{label}'!")
            return

        with self._lock:
            self._current_goal_handle = goal_handle

        self.get_logger().info(f"✓ Nav2 accepted route for '{label}'. Robot following Topology lanes...")
        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            threading.Event().wait(0.1)

        if result_future.done() and not result_future.cancelled():
            res = result_future.result()
            if res and res.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"🎉 Successfully arrived at destination for '{label}'!")

        with self._lock:
            self._current_goal_handle = None

    def on_rviz_pose(self, msg: PoseStamped):
        cx = float(msg.pose.position.x)
        cy = float(msg.pose.position.y)

        vertices, lanes = self.load_graph()
        if not vertices or not lanes:
            self.get_logger().error("No Topology Graph loaded!")
            return

        self.update_robot_pose_from_tf()
        start_idx = self.find_closest_vertex_idx(self.robot_x, self.robot_y, vertices)

        # 1. Tìm trạm đỗ gần nhất điểm click
        target_wp_idx = self.find_closest_vertex_idx(cx, cy, vertices)
        target_wp_v = vertices[target_wp_idx]
        dist_to_wp = math.hypot(cx - float(target_wp_v[0]), cy - float(target_wp_v[1]))
        wp_name = target_wp_v[2].get('name', f'wp_{target_wp_idx}')

        # 2. Tìm điểm chiếu vuông góc gần nhất trên các làn Topology
        best_proj, proj_dist, best_lane, best_t = self.find_closest_lane_projection(cx, cy, vertices, lanes)

        # Nếu click rất gần 1 trạm đỗ (trong bán kính 0.4m) -> đi chuẩn trạm đỗ RMF
        if dist_to_wp < 0.4:
            self.get_logger().info(f"Clicked near station '{wp_name}' (dist={dist_to_wp:.2f}m). Navigating to station...")
            path_indices = self.find_shortest_topology_path(start_idx, target_wp_idx, vertices, lanes)

            route_pts = []
            for idx in path_indices:
                route_pts.append((float(vertices[idx][0]), float(vertices[idx][1])))

            self.start_navigation_async(route_pts, f"Station {wp_name}")
            return

        # Nếu click trên hoặc lân cận làn đường (Lanes):
        u_idx, v_idx = best_lane
        target_node_idx = u_idx if best_t <= 0.5 else v_idx
        path_indices = self.find_shortest_topology_path(start_idx, target_node_idx, vertices, lanes)

        route_pts = []
        for idx in path_indices:
            route_pts.append((float(vertices[idx][0]), float(vertices[idx][1])))

        route_pts.append((best_proj[0], best_proj[1]))

        if proj_dist > 0.05:
            route_pts.append((cx, cy))

        self.get_logger().info(
            f"Clicked on/near Lane ({u_idx}-{v_idx}). "
            f"Following Topology lane to ({best_proj[0]:.2f}, {best_proj[1]:.2f}) -> Clicked Point ({cx:.2f}, {cy:.2f})."
        )

        self.start_navigation_async(route_pts, f"Point near Lane ({cx:.2f}, {cy:.2f})")

    def run(self):
        target = self.get_parameter('waypoint').value
        if not target and len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
            target = sys.argv[1]

        if not target:
            self.get_logger().info("Topology Smart Async Navigator Ready! Click consecutive goals anytime on RViz2!")
            return

        vertices, lanes = self.load_graph()
        if not vertices:
            self.get_logger().error("No Topology Graph loaded!")
            return

        target_idx = None
        for i, v in enumerate(vertices):
            name = v[2].get('name', '') if len(v) > 2 else ''
            if name == target:
                target_idx = i
                break

        if target_idx is None:
            names = [v[2].get('name', '?') for v in vertices if len(v) > 2 and v[2].get('name')]
            self.get_logger().error(f"Cannot find waypoint '{target}'. Available waypoints: {names}")
            return

        self.update_robot_pose_from_tf()
        start_idx = self.find_closest_vertex_idx(self.robot_x, self.robot_y, vertices)

        path_indices = self.find_shortest_topology_path(start_idx, target_idx, vertices, lanes)
        route_pts = [(float(vertices[idx][0]), float(vertices[idx][1])) for idx in path_indices]
        self.start_navigation_async(route_pts, f"Station {target}")


def main():
    rclpy.init()
    node = GotoWaypoint()
    node.run()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
