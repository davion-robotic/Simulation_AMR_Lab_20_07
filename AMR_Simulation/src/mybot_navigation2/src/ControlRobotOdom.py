#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import struct
import time


class TeleopUART(Node):

    def __init__(self):
        super().__init__('teleop_uart')

        # ===== UART =====
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        time.sleep(2)

        # ===== data =====  
        self.a = 0.0  # linear.x
        self.b = 0.0  # angular.z

        # ===== subscribe cmd_vel =====
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # ===== gửi mỗi 500ms =====
        self.create_timer(0.5, self.send_data)

    def cmd_callback(self, msg):
        self.a = msg.linear.x
        self.b = msg.angular.z

        self.get_logger().info(f"Recv cmd_vel → a={self.a:.2f}, b={self.b:.2f}")

    def send_data(self):
        data = struct.pack('<ff', self.a, self.b)  # little-endian
        self.ser.write(data)

        self.get_logger().info(f"Sent bytes: {data.hex()}")


def main(args=None):
    rclpy.init(args=args)

    node = TeleopUART()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()