#!/usr/bin/env python3
"""
stop_timer_relay — timed stop gate for the collision stop zone only.

Pipeline:
  nav2 /cmd_vel → collision_monitor_stop → /cmd_vel_stop_out
                                                  ↓
                              stop_timer_relay (also reads /cmd_vel)
                                                  ↓
                                           /cmd_vel_relay
                                                  ↓
                               collision_monitor_slow  ← always active
                                                  ↓
                                           /cmd_vel_slowed → velocity_smoother → robot

State machine  (slowdown polygons are NEVER affected — they are downstream):

  CLEAR    : normal — pass /cmd_vel_stop_out through.
               obstacle enters 0.5 m stop zone          → STOPPED

  STOPPED  : publish zero for STOP_SEC.
               timer done                               → DISABLED

  DISABLED : stop zone fully ignored for DISABLE_SEC; robot moves freely.
               publish original /cmd_vel (slowdown still applies downstream).
               timer done + object still inside         → STOPPED  (loop)
               timer done + zone clear                  → CLEAR    (re-arm)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

STOP_SEC    = 5.0   # seconds to hold zero after obstacle enters stop zone
DISABLE_SEC = 5.0   # seconds to ignore the stop zone after the hold
TICK_HZ     = 20.0


class StopTimerRelay(Node):

    def __init__(self):
        super().__init__('stop_timer_relay')

        self._latest_nav2_cmd = Twist()
        self._latest_stop_out = Twist()

        # obstacle presence in stop zone — only refreshed while nav2 is moving
        # so brief nav2 replanning pauses do not flip it to False incorrectly
        self._obstacle_in_zone = False

        self._state       = 'CLEAR'
        self._phase_start = 0.0

        self.create_subscription(Twist, 'cmd_vel',          self._nav2_cb,     10)
        self.create_subscription(Twist, 'cmd_vel_stop_out', self._stop_out_cb, 10)

        self._pub = self.create_publisher(Twist, 'cmd_vel_relay', 10)
        self.create_timer(1.0 / TICK_HZ, self._tick)

    # ------------------------------------------------------------------ #

    def _nav2_cb(self, msg: Twist):
        self._latest_nav2_cmd = msg

    def _stop_out_cb(self, msg: Twist):
        self._latest_stop_out = msg

    def _nav2_moving(self) -> bool:
        cmd = self._latest_nav2_cmd
        return abs(cmd.linear.x) > 0.01 or abs(cmd.angular.z) > 0.01

    def _update_obstacle(self):
        """Update stop-zone knowledge only while nav2 is commanding movement."""
        if not self._nav2_moving():
            return
        out = self._latest_stop_out
        self._obstacle_in_zone = (abs(out.linear.x) < 0.01 and abs(out.angular.z) < 0.01)

    # ------------------------------------------------------------------ #

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._update_obstacle()
        elapsed = now - self._phase_start

        if self._state == 'CLEAR':
            self._pub.publish(self._latest_stop_out)
            if self._obstacle_in_zone:
                self._state       = 'STOPPED'
                self._phase_start = now
                self.get_logger().info('Stop zone triggered — holding %.0f s' % STOP_SEC)

        elif self._state == 'STOPPED':
            self._pub.publish(Twist())
            if elapsed >= STOP_SEC:
                self._state       = 'DISABLED'
                self._phase_start = now
                self.get_logger().info(
                    'Stop done — disabling stop zone for %.0f s' % DISABLE_SEC)

        elif self._state == 'DISABLED':
            self._pub.publish(self._latest_nav2_cmd)
            if elapsed >= DISABLE_SEC:
                if self._obstacle_in_zone:
                    self._state       = 'STOPPED'
                    self._phase_start = now
                    self.get_logger().info(
                        'Object still present — stopping again')
                else:
                    self._state = 'CLEAR'
                    self.get_logger().info(
                        'Zone clear — re-armed')


def main(args=None):
    rclpy.init(args=args)
    node = StopTimerRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
