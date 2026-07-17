#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped

class FixPointCloudNode(Node):
    def __init__(self):
        super().__init__('fix_pc_node')
        # Thay 'input_cloud' bằng tên topic đang bị lỗi của bạn
        self.sub = self.create_subscription(PointCloud2, '/depth_pcl', self.callback, 10)
        # Topic đã được sửa
        self.pub = self.create_publisher(PointCloud2, 'fixed_cloud', 10)

    def callback(self, msg):
        # Tạo một Transform giả lập để xoay từ Optical về Standard
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'fixed_link'

        # Ma trận biến đổi: Roll = -90 độ, Yaw = -90 độ
        # Chuyển đổi sang Quaternion (x, y, z, w)
        t.transform.rotation.x = -0.5
        t.transform.rotation.y =  0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w =  0.5

        # Dùng thư viện có sẵn của ROS 2 để xoay toàn bộ PointCloud2
        fixed_cloud = do_transform_cloud(msg, t)
        
        # Gán lại frame cho chuẩn chỉnh
        fixed_cloud.header.frame_id = 'base_link'
        
        # Publish ra ngoài
        self.pub.publish(fixed_cloud)

def main(args=None):
    rclpy.init(args=args)
    node = FixPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()