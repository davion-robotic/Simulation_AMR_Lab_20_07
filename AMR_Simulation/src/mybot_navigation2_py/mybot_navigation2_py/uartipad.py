import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import serial
import threading


class UARTBridge(Node):

    def __init__(self):
        super().__init__('uart_bridge')

        # ================= PUB (RX → ROS) =================
        self.publisher_ = self.create_publisher(
            Int32MultiArray,
            '/uart_cmd',
            10
        )

        # ================= SUB (ROS → TX) =================
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/nav2_status_code',
            self.status_callback,
            10
        )

        # ================= SERIAL =================
        try:
            self.ser = serial.Serial('/dev/Monitor', 115200, timeout=0.1)
            self.get_logger().info("📡 UART connected (/dev/Monitor)")
        except Exception as e:
            self.get_logger().error(f"❌ Serial error: {e}")
            exit(1)

        self.buffer = ""

        # ================= RX THREAD =================
        self.thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.thread.start()

    # ================= SERIAL RX =================
    def serial_loop(self):
        while rclpy.ok():
            try:
                data = self.ser.read(64)
                if data:
                    self.buffer += data.decode(errors='ignore')
                    self.extract_packets()
            except Exception as e:
                self.get_logger().error(f"❌ Read error: {e}")

    def extract_packets(self):
        while True:
            start = self.buffer.find('!')
            end = self.buffer.find('#', start + 1)

            if start == -1 or end == -1:
                break

            packet = self.buffer[start:end + 1]
            self.buffer = self.buffer[end + 1:]

            self.handle_packet(packet)

    def handle_packet(self, packet: str):

        if not (packet.startswith('!') and packet.endswith('#')):
            return

        try:
            payload = packet[1:-1].strip()   # "12"

            if len(payload) < 2:
                return

            header = int(payload[0])
            value  = int(payload[1])

            msg = Int32MultiArray()
            msg.data = [header, value]

            self.publisher_.publish(msg)

            self.get_logger().info(f"📥 UART RX → [{header}, {value}]")

        except Exception as e:
            self.get_logger().warn(f"❌ Parse error: {e}")                                

    # ================= SERIAL TX =================
    def status_callback(self, msg: Int32MultiArray):

        if len(msg.data) < 2:
            return

        header, code = msg.data

        try:
            # Build readable string with spaces
            packet = f"!{header}{code}#"

            self.ser.write(packet.encode('utf-8'))

            self.get_logger().info(f"📤 UART TX → {packet}")

        except Exception as e:
            self.get_logger().error(f"❌ Write error: {e}")
    # def status_callback(self, msg: Int32MultiArray):

    #     try:
    #         packet = "hello"

    #         self.ser.write(packet.encode('utf-8'))

    #         self.get_logger().info(f"📤 UART TX → {packet}")

    #     except Exception as e:
    #         self.get_logger().error(f"❌ Write error: {e}")


def main():
    rclpy.init()
    node = UARTBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()