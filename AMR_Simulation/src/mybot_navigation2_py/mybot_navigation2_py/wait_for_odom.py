import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class WaitForOdom(Node):

    def __init__(self):
        super().__init__('wait_for_odom')

        self.pub = self.create_publisher(Bool, '/odom_ready', 10)

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.cb,
            10
        )

        self.sent = False
        self.get_logger().info("⏳ Waiting for /odom...")

    def cb(self, msg):
        if not self.sent:
            self.sent = True
            self.get_logger().info("✅ /odom received → publishing /odom_ready")

            self.pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = WaitForOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()