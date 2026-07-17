#!/usr/bin/env python3
"""
robot_publisher.py
-------------------
Node ROS2 mo phong robot, publish du lieu len 5 topic chuan:
  - /map          (nav_msgs/OccupancyGrid)
  - /scan         (sensor_msgs/LaserScan)
  - /tf           (tf2_msgs/TFMessage)   - odom -> base_link
  - /tf_static    (tf2_msgs/TFMessage)   - base_link -> laser (khong doi theo thoi gian)
  - /odom         (nav_msgs/Odometry)

Chay:
    python3 robot_publisher.py
(can source ROS2 truoc: source /opt/ros/<distro>/setup.bash)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Header


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RobotPublisher(Node):
    def __init__(self):
        super().__init__('robot_publisher')

        # ---- Publisher /map ----
        # map thuong khong doi nhieu, publish 1 lan / vai giay la du
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.map_timer = self.create_timer(5.0, self.publish_map)

        # ---- Publisher /scan ----
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.scan_timer = self.create_timer(0.1, self.publish_scan)  # 10Hz

        # ---- Publisher /tf (dynamic: odom -> base_link) ----
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.tf_timer = self.create_timer(0.05, self.publish_tf)  # 20Hz

        # ---- Publisher /tf_static (base_link -> laser) ----
        static_qos = QoSProfile(depth=1)
        static_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        static_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', static_qos)
        self.publish_tf_static()  # chi can publish 1 lan

        # ---- Publisher /odom ----
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_timer = self.create_timer(0.1, self.publish_odom)  # 10Hz

        # trang thai robot gia lap (chay vong tron)
        self.t0 = time.time()

        self.get_logger().info('robot_publisher da khoi dong, dang publish /map /scan /tf /tf_static /odom')

    # -----------------------------------------------------------------
    def robot_pose(self):
        """Gia lap robot di chuyen theo hinh tron ban kinh 1m."""
        t = time.time() - self.t0
        radius = 1.0
        omega = 0.3
        x = radius * math.cos(omega * t)
        y = radius * math.sin(omega * t)
        yaw = omega * t + math.pi / 2.0
        return x, y, yaw

    # -----------------------------------------------------------------
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        meta = MapMetaData()
        meta.resolution = 0.05
        width, height = 100, 100
        meta.width = width
        meta.height = height
        meta.origin.position.x = -2.5
        meta.origin.position.y = -2.5
        meta.origin.orientation.w = 1.0
        msg.info = meta

        # tao map gia: toan bo free (0), vien la occupied (100)
        data = [0] * (width * height)
        for i in range(width):
            data[i] = 100
            data[(height - 1) * width + i] = 100
        for j in range(height):
            data[j * width] = 100
            data[j * width + (width - 1)] = 100
        msg.data = data

        self.map_pub.publish(msg)
        self.get_logger().debug('Published /map')

    # -----------------------------------------------------------------
    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        num_readings = 360
        msg.angle_increment = (msg.angle_max - msg.angle_min) / num_readings
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # gia lap khoang cach ngau nhien quanh 2m, co nhieu nho
        import random
        msg.ranges = [2.0 + random.uniform(-0.1, 0.1) for _ in range(num_readings)]
        msg.intensities = [1.0] * num_readings

        self.scan_pub.publish(msg)

    # -----------------------------------------------------------------
    def publish_tf(self):
        x, y, yaw = self.robot_pose()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(yaw)

        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        self.tf_pub.publish(tf_msg)

    # -----------------------------------------------------------------
    def publish_tf_static(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.w = 1.0

        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        self.tf_static_pub.publish(tf_msg)
        self.get_logger().info('Published /tf_static (mot lan)')

    # -----------------------------------------------------------------
    def publish_odom(self):
        x, y, yaw = self.robot_pose()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        # van toc gia lap
        msg.twist.twist.linear.x = 0.3
        msg.twist.twist.angular.z = 0.3

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()