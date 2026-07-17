#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
import struct
import math


class OdomUART(Node):

    def __init__(self):
        super().__init__('odom_uart')

        # ===== UART =====
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # ===== ROS =====
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        # ===== state =====
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.buffer = bytearray()

        # ===== timers =====
        self.create_timer(0.01, self.read_data)     # đọc nhanh
        self.create_timer(0.5, self.publish_odom)  # publish chậm

        self.get_logger().info("Odom UART node started")

    def read_data(self):
        # đọc toàn bộ data có sẵn
        data = self.ser.read(self.ser.in_waiting)
        if data:
            self.buffer += data

        while True:
            # cần đủ 14 bytes
            if len(self.buffer) < 14:
                return

            # tìm header 0xAA
            if self.buffer[0] != 0xAA:
                self.buffer.pop(0)
                continue

            # đủ frame chưa
            if len(self.buffer) < 14:
                return

            frame = self.buffer[:14]

            # check end byte
            if frame[-1] != 0x0D:
                self.get_logger().warn("Bad frame (wrong end byte)")
                self.buffer.pop(0)
                continue

            try:
                payload = frame[1:13]
                self.x, self.y, self.theta = struct.unpack('<fff', payload)

            except Exception as e:
                self.get_logger().error(f"Unpack error: {e}")
                self.buffer.pop(0)
                continue

            # log raw (debug nhẹ)
            self.get_logger().debug(f"Frame OK: {frame.hex()}")

            # bỏ frame đã xử lý
            self.buffer = self.buffer[14:]

    def publish_odom(self):
        msg = Odometry()

        # ===== position =====
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # ===== orientation (theta → quaternion) =====
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # ===== publish =====
        self.pub.publish(msg)

        # ===== log =====
        self.get_logger().info(
            f"Odom → x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = OdomUART()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info("Shutting down...")
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()