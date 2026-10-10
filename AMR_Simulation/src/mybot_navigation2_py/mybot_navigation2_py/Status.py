import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import Int32MultiArray


class Nav2LogParser(Node):

    def __init__(self):
        super().__init__('nav2_log_parser')

        self.get_logger().info("👀 Parallel Log Parser Started")

        # Publisher: [header, code]
        self.publisher_ = self.create_publisher(
            Int32MultiArray,
            '/nav2_status_code',
            10
        )

        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            100
        )

        # 🔥 Track state PER HEADER (important!)
        self.last_state = {
            1: None,  # Nav2
            2: None   # Collision
        }

    def log_callback(self, msg: Log):

        text = msg.msg
        node_name = msg.name

        header = None
        code = None

        # =========================
        # NAV2
        # =========================
        if "controller_server" in node_name:
            header = 1

            if "Passing new path to controller" in text:
                code = 2

            elif "Reached the goal!" in text:
                code = 3

            elif "Received a goal" in text:
                code = 1

        # =========================
        # COLLISION
        # =========================
        elif "collision_monitor" in node_name:
            header = 2

            if "slowdown" in text:
                code = 2

            elif "stop" in text:
                code = 3

            elif "continue normal operation" in text:
                code = 1

        # =========================
        # PUBLISH (parallel-safe)
        # =========================
        if header is not None and code is not None:

            # only block duplicates from SAME source
            if self.last_state[header] != code:
                self.last_state[header] = code

                msg_out = Int32MultiArray()
                msg_out.data = [header, code]

                self.publisher_.publish(msg_out)

                self.get_logger().info(
                    f"📡 Published: header={header}, code={code}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2LogParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()