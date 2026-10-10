import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess


class Nav2Starter(Node):

    def __init__(self):
        super().__init__('nav2_starter')

        self.started = False

        self.sub = self.create_subscription(
            Bool,
            '/odom_ready',
            self.cb,
            10
        )

        self.get_logger().info("⏳ Waiting for /odom_ready...")

    def cb(self, msg):
        if msg.data and not self.started:
            self.started = True

            self.get_logger().info("🚀 Starting Nav2 safely...")

            subprocess.Popen([
                'ros2', 'launch',
                'nav2_bringup', 'bringup_launch.py',
                'map:=/home/amr/my_bot/src/mybot_navigation2_py/map/my_map3_edited.yaml',
                'params_file:=/home/amr/my_bot/src/mybot_navigation2_py/param/mybot2.yaml',
                'use_sim_time:=false',
                'autostart:=true',
                'use_composition:=True'
            ])


def main():
    rclpy.init()
    node = Nav2Starter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()