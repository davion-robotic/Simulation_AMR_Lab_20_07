#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class TaskControllerNode(Node):
    """
    Node to handle task control commands from UI (via rosbridge_websocket or ROS2 topic):
      - PAUSE: Temporarily stop the current navigation task
      - RESUME: Continue the paused navigation task
      - CANCEL: Completely cancel the current navigation task and stop the robot
    """

    def __init__(self):
        super().__init__('task_controller_node')

        # Parameters
        self.declare_parameter('cmd_topic', '/task_controller/command')
        self.declare_parameter('status_topic', '/task_controller/status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('nav2_action_name', 'navigate_to_pose')

        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        nav2_action = self.get_parameter('nav2_action_name').get_parameter_value().string_value

        # Robot/Task States: 'IDLE', 'RUNNING', 'PAUSED', 'CANCELLED'
        self.state = 'IDLE'
        self.current_goal_handle = None
        self.saved_goal_pose = None

        # Subscribers
        self.cmd_sub = self.create_subscription(
            String,
            cmd_topic,
            self.command_callback,
            10
        )

        # Optional: listen to external goal submissions to track active navigation goals
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_subscription if False else self.create_publisher(
            String,
            status_topic,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        # Action Client for Nav2 NavigateToPose
        self.nav2_client = ActionClient(self, NavigateToPose, nav2_action)

        # Periodic status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_periodic_status)

        self.get_logger().info('=============================================')
        self.get_logger().info('🚀 Task Controller Node initialized successfully!')
        self.get_logger().info(f'📡 Listening on: {cmd_topic}')
        self.get_logger().info(f'📊 Publishing status to: {status_topic}')
        self.get_logger().info('=============================================')

    def publish_periodic_status(self):
        """Periodically broadcast the current task controller state."""
        self.publish_status(self.state, f"Controller in {self.state} state")

    def publish_status(self, state: str, message: str = ""):
        """Publish JSON status to UI."""
        payload = {
            "state": state,
            "message": message,
            "has_saved_goal": self.saved_goal_pose is not None,
            "timestamp": self.get_clock().now().to_msg().sec
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def stop_robot_motion(self):
        """Immediately publish 0 velocities on /cmd_vel."""
        zero_twist = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(zero_twist)

    def goal_pose_callback(self, msg: PoseStamped):
        """Track new goal sent via /goal_pose."""
        self.saved_goal_pose = msg
        self.state = 'RUNNING'
        self.get_logger().info(f'📍 New goal received at x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.publish_status(self.state, "New goal received")

    def parse_command_string(self, raw_data: str) -> str:
        """Extract the command keyword from string or JSON format."""
        raw_clean = raw_data.strip()
        # If payload is JSON format (e.g. {"data": "PAUSE"} or {"command": "PAUSE"})
        if raw_clean.startswith('{') and raw_clean.endswith('}'):
            try:
                parsed = json.loads(raw_clean)
                if 'data' in parsed:
                    return str(parsed['data']).strip().upper()
                if 'command' in parsed:
                    return str(parsed['command']).strip().upper()
                if 'cmd' in parsed:
                    return str(parsed['cmd']).strip().upper()
            except Exception:
                pass
        return raw_clean.upper()

    def command_callback(self, msg: String):
        raw_text = msg.data
        cmd = self.parse_command_string(raw_text)

        self.get_logger().info('=' * 60)
        self.get_logger().info('📥 [UI INCOMING DATA] Nhận được gói tin từ UI!')
        self.get_logger().info(f'   ├─ Topic:       {self.get_parameter("cmd_topic").get_parameter_value().string_value}')
        self.get_logger().info(f'   ├─ Dữ liệu thô: "{raw_text}"')
        self.get_logger().info(f'   ├─ Lệnh xử lý:  "{cmd}"')
        self.get_logger().info(f'   └─ State trước: {self.state}')
        self.get_logger().info('=' * 60)

        if cmd == 'PAUSE':
            self.handle_pause()
        elif cmd == 'RESUME':
            self.handle_resume()
        elif cmd == 'CANCEL':
            self.handle_cancel()
        else:
            self.get_logger().warn(f'⚠️ [UI WARNING] Lệnh không hợp lệ: "{cmd}". Các lệnh hợp lệ: PAUSE, RESUME, CANCEL')
            self.publish_status(self.state, f"Ignored unknown command: {cmd}")

    def handle_pause(self):
        """Handle PAUSE command."""
        self.get_logger().info('⏸️ Processing PAUSE command...')
        
        # 1. Stop physical motion immediately
        self.stop_robot_motion()

        # 2. Cancel active Nav2 goal if running
        if self.current_goal_handle is not None and self.state == 'RUNNING':
            self.get_logger().info('Canceling current Nav2 goal while keeping saved pose for resume...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.pause_cancel_done)
        
        self.state = 'PAUSED'
        self.get_logger().info('⏸️ Task is now PAUSED.')
        self.publish_status('PAUSED', 'Task paused successfully')

    def pause_cancel_done(self, future):
        self.get_logger().info('Nav2 goal canceled for PAUSE.')
        self.stop_robot_motion()

    def handle_resume(self):
        """Handle RESUME command."""
        self.get_logger().info('▶️ Processing RESUME command...')

        if self.state != 'PAUSED':
            self.get_logger().warn(f'⚠️ Cannot RESUME when current state is "{self.state}". (Expected PAUSED)')
            self.publish_status(self.state, f"Cannot resume from state {self.state}")
            return

        if self.saved_goal_pose is None:
            self.get_logger().warn('⚠️ No saved goal pose to resume.')
            self.state = 'IDLE'
            self.publish_status(self.state, "No goal to resume")
            return

        # Send saved goal pose back to Nav2
        self.get_logger().info('Sending saved goal to Nav2 to resume navigation...')
        self.send_nav2_goal(self.saved_goal_pose)
        self.state = 'RUNNING'
        self.publish_status('RUNNING', 'Task resumed')

    def handle_cancel(self):
        """Handle CANCEL command."""
        self.get_logger().info('⏹️ Processing CANCEL command...')

        # 1. Stop physical motion immediately
        self.stop_robot_motion()

        # 2. Cancel active Nav2 goal
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling active Nav2 goal...')
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        # 3. Reset saved goal and state
        self.saved_goal_pose = None
        self.state = 'IDLE'
        self.get_logger().info('⏹️ Task CANCELLED. State is now IDLE.')
        self.publish_status('CANCELLED', 'Task has been cancelled')

    def send_nav2_goal(self, pose: PoseStamped):
        """Send goal to Nav2 NavigateToPose action server."""
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('⚠️ Nav2 NavigateToPose action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Sending goal request to Nav2...')
        send_goal_future = self.nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav2_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal was rejected by Nav2!')
            self.state = 'IDLE'
            self.publish_status('IDLE', 'Goal rejected by Nav2')
            return

        self.get_logger().info('✅ Goal accepted by Nav2.')
        self.current_goal_handle = goal_handle
        self.state = 'RUNNING'

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Keep track of distance remaining or current position if needed
        pass

    def nav2_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🎯 Nav2 Goal SUCCEEDED!')
            self.saved_goal_pose = None
            self.current_goal_handle = None
            self.state = 'IDLE'
            self.publish_status('COMPLETED', 'Navigation goal reached successfully')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('🛑 Nav2 Goal CANCELED.')
            if self.state != 'PAUSED':
                self.saved_goal_pose = None
                self.current_goal_handle = None
                self.state = 'IDLE'
                self.publish_status('CANCELLED', 'Navigation goal was canceled')
        else:
            self.get_logger().warn(f'⚠️ Nav2 Goal finished with status: {status}')
            self.saved_goal_pose = None
            self.current_goal_handle = None
            self.state = 'IDLE'
            self.publish_status('IDLE', f'Goal finished with status {status}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
