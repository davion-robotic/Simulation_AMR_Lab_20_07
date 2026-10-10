import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32MultiArray

import json
import os
import math


class WaypointRunner(Node):

    def __init__(self):
        super().__init__('waypoint_runner')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.base_path = '/home/amr/my_bot/src/mybot_navigation2_py/param'

        self.home = None
        self.waypoint_list = []

        # ================= LOAD DEFAULT FILE =================
        self.load_file("T10")

        # send HOME once at start
        self.get_logger().info("🏠 Sending HOME after load")
        self.send_goal(self.home)

        # ================= SUBSCRIBER =================
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/uart_cmd',
            self.uart_callback,
            10
        )

        self.get_logger().info("📡 Waiting for /uart_cmd")

    # ================= LOAD FILE =================
    def load_file(self, filename):
        path = os.path.join(self.base_path, filename + ".json")

        if not os.path.exists(path):
            self.get_logger().error(f"❌ File {filename} not found")
            exit(1)

        with open(path, 'r') as f:
            data = json.load(f)

        self.home = data["home"]
        self.waypoint_list = data["waypoints"]

        self.get_logger().info(f"✅ Loaded {filename}")

    # ================= UART CALLBACK =================
    def uart_callback(self, msg: Int32MultiArray):

        if len(msg.data) < 2:
            return

        header, value = msg.data

        self.get_logger().info(f"📥 UART RX → header={header}, value={value}")

        # only accept header 0x01
        if header != 0x05:
            return

        # HOME
        if value == 0x00:
            self.get_logger().info("🏠 Going HOME")
            self.send_goal(self.home)
            return

        # WAYPOINT
        index = value - 1

        if 0 <= index < len(self.waypoint_list):
            wp = self.waypoint_list[index]
            self.get_logger().info(f"📍 Going to waypoint: {wp['name']}")
            self.send_goal(wp["pose"])
        else:
            self.get_logger().warn("❌ Waypoint index out of range")

    # ================= SEND GOAL =================
    def send_goal(self, pose_dict):
        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = pose_dict["x"]
        pose.pose.position.y = pose_dict["y"]
        pose.pose.position.z = pose_dict.get("z", 0.0)

        pose.pose.orientation.x = pose_dict["qx"]
        pose.pose.orientation.y = pose_dict["qy"]
        pose.pose.orientation.z = pose_dict["qz"]
        pose.pose.orientation.w = pose_dict["qw"]

        goal.pose = pose

        yaw = self.quaternion_to_yaw(pose.pose.orientation)
        deg = math.degrees(yaw)

        self.get_logger().info(
            f"🚀 Sending Goal | Heading: {deg:.2f}° | "
            f"x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}"
        )

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # ================= CALLBACKS =================
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return

        self.get_logger().info("✅ Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("🎯 Goal reached")

    # ================= UTILS =================
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = WaypointRunner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()