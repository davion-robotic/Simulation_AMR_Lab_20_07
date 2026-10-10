import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import json
import sys
import select
import termios
import tty
import os
import math

import tf2_ros
import tf_transformations


class WaypointManager(Node):

    def __init__(self):
        super().__init__('waypoint_manager')

        # ================= TF BUFFER (GetCurrentPose style) =================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.home_pose = None
        self.waypoints = []

        self.base_path = '/home/amr/my_bot/src/mybot_navigation2_py/param/'

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(
            "h = toggle HOME | s = save waypoint | w = write file | Ctrl+C = exit"
        )

        self.timer = self.create_timer(0.1, self.check_keyboard)

    # ================= GET CURRENT POSE (LIKE GetCurrentPose BT) =================
    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )

            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            return pose

        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}")
            return None

    # ================= KEYBOARD =================
    def check_keyboard(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()

            if key == 'h':
                self.toggle_home()

            elif key == 's':
                self.save_named_waypoint()

            elif key == 'w':
                self.write_file()

    # ================= QUAT → YAW =================
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_degrees(self, yaw):
        return math.degrees(yaw)

    # ================= POSE =================
    def extract_pose(self, pose_msg):
        p = pose_msg.pose

        return {
            "x": p.position.x,
            "y": p.position.y,
            "z": p.position.z,
            "qx": p.orientation.x,
            "qy": p.orientation.y,
            "qz": p.orientation.z,
            "qw": p.orientation.w
        }

    # ================= HOME =================
    def toggle_home(self):
        pose_msg = self.get_current_pose()
        if pose_msg is None:
            self.get_logger().warn("No TF pose yet!")
            return

        if self.home_pose is None:
            self.home_pose = self.extract_pose(pose_msg)

            q = pose_msg.pose.orientation
            yaw = self.quaternion_to_yaw(q)
            deg = self.yaw_to_degrees(yaw)

            self.get_logger().info(f"🏠 Home SAVED | Heading: {deg:.2f}°")
        else:
            self.home_pose = None
            self.get_logger().info("❌ Home REMOVED")

    # ================= SAVE WAYPOINT =================
    def save_named_waypoint(self):
        pose_msg = self.get_current_pose()
        if pose_msg is None:
            self.get_logger().warn("No TF pose yet!")
            return

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        try:
            name = input("\nEnter waypoint name: ").strip()

            if not name:
                self.get_logger().warn("Empty name, cancelled")
                return

            for wp in self.waypoints:
                if wp["name"] == name:
                    self.get_logger().warn("Name already exists!")
                    return

            pose_data = self.extract_pose(pose_msg)

            waypoint = {
                "name": name,
                "pose": pose_data
            }

            self.waypoints.append(waypoint)

            q = pose_msg.pose.orientation
            yaw = self.quaternion_to_yaw(q)
            deg = self.yaw_to_degrees(yaw)

            self.get_logger().info(
                f"✅ Saved '{name}' | Heading: {deg:.2f}° | "
                f"x={pose_data['x']:.3f}, y={pose_data['y']:.3f}"
            )

        finally:
            tty.setcbreak(sys.stdin.fileno())

    # ================= WRITE FILE =================
    def write_file(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        try:
            filename = input("\nEnter filename: ").strip()

            if not filename:
                self.get_logger().warn("Empty filename")
                return

            full_path = os.path.join(self.base_path, filename + ".json")
            os.makedirs(os.path.dirname(full_path), exist_ok=True)

            data = {
                "home": self.home_pose,
                "waypoints": self.waypoints
            }

            with open(full_path, 'w') as f:
                json.dump(data, f, indent=4)

            self.get_logger().info(f"💾 Saved TF-based file: {full_path}")

        finally:
            tty.setcbreak(sys.stdin.fileno())

    # ================= CLEANUP =================
    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main():
    rclpy.init()
    node = WaypointManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()