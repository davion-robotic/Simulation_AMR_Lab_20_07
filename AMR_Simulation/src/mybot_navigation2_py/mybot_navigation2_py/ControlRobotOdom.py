#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

import serial
import struct
import math
import time


# QoS for /motor_enable — reliable + transient-local so a late subscriber
# still receives the last published value (e.g. from the launch-file publisher)
MOTOR_ENABLE_QOS = QoSProfile(
    reliability = ReliabilityPolicy.RELIABLE,
    durability  = DurabilityPolicy.TRANSIENT_LOCAL,
    history     = HistoryPolicy.KEEP_LAST,
    depth       = 1,
)


class UARTBridge(Node):

    def __init__(self):
        super().__init__('uart_bridge')

        # ===== UART =====
        self.ser = serial.Serial('/dev/stm32', 115200, timeout=0.001)
        time.sleep(2)

        # ===== subscriptions =====
        self.create_subscription(Twist, '/cmd_vel_control', self.cmd_callback, 10)
        self.create_subscription(Bool,  '/motor_enable',    self.enable_callback,
                                 MOTOR_ENABLE_QOS)

        # ===== publishers =====
        self.odom_pub       = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== state =====
        self.a      = 0.0
        self.b      = 0.0
        self.enable = 1     # 1 = motors on, 0 = free wheels

        # ===== receive buffer =====
        self.buffer = bytearray()

        # ===== main loop =====
        self.create_timer(0.1, self.loop)

        self.get_logger().info(
            'uart_bridge ready — subscribed to /cmd_vel_control and /motor_enable')

    # ------------------------------------------------------------------ #
    # subscriptions
    # ------------------------------------------------------------------ #

    def cmd_callback(self, msg: Twist):
        self.a = msg.linear.x
        self.b = msg.angular.z

    def enable_callback(self, msg: Bool):
        self.enable = 1 if msg.data else 0
        self.get_logger().info(
            f'[motor_enable] Motors -> {"ENABLED" if self.enable else "DISABLED"}')

    # ------------------------------------------------------------------ #
    # main loop
    # ------------------------------------------------------------------ #

    def loop(self):
        self.send_cmd()
        self.read_odom()

    # ------------------------------------------------------------------ #
    # TX → STM32  (9 bytes: <ffB>)
    # ------------------------------------------------------------------ #

    def send_cmd(self):
        try:
            u = self.a if self.enable else 0.0
            r = self.b if self.enable else 0.0
            self.ser.write(struct.pack('<ffB', u, r, self.enable))
        except Exception as e:
            self.get_logger().error(f'UART TX error: {e}')

    # ------------------------------------------------------------------ #
    # RX ← STM32  (14-byte frame: 0xAA [12 bytes float*3] 0x0D)
    # ------------------------------------------------------------------ #

    def read_odom(self):
        raw = self.ser.read(self.ser.in_waiting)
        if raw:
            self.buffer += raw

        while len(self.buffer) >= 14:
            if self.buffer[0] != 0xAA:
                self.buffer.pop(0)
                continue

            frame = self.buffer[:14]

            if frame[-1] != 0x0D:
                self.buffer.pop(0)
                continue

            try:
                x, y, theta = struct.unpack('<fff', frame[1:13])
            except Exception as e:
                self.get_logger().error(f'Unpack error: {e}')
                self.buffer.pop(0)
                continue

            self.publish_odom(x, y, theta)
            self.buffer = self.buffer[14:]

    # ------------------------------------------------------------------ #
    # publish odom + TF
    # ------------------------------------------------------------------ #

    def publish_odom(self, x, y, theta):
        now = self.get_clock().now().to_msg()
        qz  = math.sin(theta / 2.0)
        qw  = math.cos(theta / 2.0)

        msg = Odometry()
        msg.header.stamp            = now
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base_link'
        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.odom_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp               = now
        t.header.frame_id            = 'odom'
        t.child_frame_id             = 'base_link'
        t.transform.translation.x    = x
        t.transform.translation.y    = y
        t.transform.translation.z    = 0.0
        t.transform.rotation.z       = qz
        t.transform.rotation.w       = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down uart_bridge')
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
