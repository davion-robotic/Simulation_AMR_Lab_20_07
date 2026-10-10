#!/usr/bin/env python3
"""Lifecycle manager for mapping - controls when SLAM actually starts."""

import json
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mybot_common.topic_names import MAPPING_SCAN_TOPIC, MAPPING_SCAN_STATUS_TOPIC
from mybot_common.command_utils import parse_command
from mybot_common.status_utils import format_status_json, create_status_payload


class MappingLifecycleManager(Node):
    def __init__(self):
        super().__init__('mapping_lifecycle_manager')
        
        self.slam_process = None
        self.slam_lock = threading.Lock()
        
        # Subscribe to mapping trigger commands
        self.subscription = self.create_subscription(
            String,
            MAPPING_SCAN_TOPIC,
            self.on_mapping_command,
            10,
        )
        
        self.status_publisher = self.create_publisher(
            String,
            MAPPING_SCAN_STATUS_TOPIC,
            10,
        )
        
        self.get_logger().info('Mapping lifecycle manager ready.')
        self.get_logger().info(f'Waiting for start/stop commands on {MAPPING_SCAN_TOPIC}')

    def on_mapping_command(self, msg: String):
        try:
            command = parse_command(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f'Invalid payload: {msg.data!r} ({exc})')
            self.publish_status({'status': 'error', 'message': str(exc)})
            return

        if command == 'start':
            self.start_mapping()
        elif command == 'stop':
            self.stop_mapping()
        else:
            self.get_logger().warning(f'Unknown command: {command}')
            self.publish_status({'status': 'error', 'message': f'Unknown command: {command}'})

    def start_mapping(self):
        with self.slam_lock:
            if self.slam_process is not None:
                self.get_logger().warning('Mapping already running.')
                self.publish_status({'status': 'warning', 'message': 'Mapping already running'})
                return

            try:
                self.get_logger().info('Starting SLAM mapping...')
                self.slam_process = subprocess.Popen(
                    ['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
                     '--ros-args',
                     '-p', 'use_sim_time:=false',
                     '-p', 'odom_frame:=odom',
                     '-p', 'base_frame:=base_link',
                     '-p', 'map_frame:=map',
                     '-p', 'scan_topic:=/scan',
                     '-p', 'map_update_interval:=2.0',
                     '-p', 'max_laser_range:=12.0',
                     '-p', 'minimum_travel_distance:=0.05',
                     '-p', 'minimum_travel_heading:=0.05'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                self.get_logger().info(f'SLAM process started with PID {self.slam_process.pid}')
                self.publish_status({'status': 'ok', 'message': 'Mapping started'})
            except Exception as e:
                self.get_logger().error(f'Failed to start mapping: {e}')
                self.slam_process = None
                self.publish_status({'status': 'error', 'message': str(e)})

    def stop_mapping(self):
        with self.slam_lock:
            if self.slam_process is None:
                self.get_logger().warning('Mapping not running.')
                self.publish_status({'status': 'warning', 'message': 'Mapping not running'})
                return

            try:
                self.get_logger().info('Stopping SLAM mapping...')
                self.slam_process.terminate()
                
                # Wait up to 5 seconds for graceful shutdown
                try:
                    self.slam_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning('Process did not terminate gracefully, killing...')
                    self.slam_process.kill()
                    self.slam_process.wait()
                
                self.get_logger().info('SLAM process stopped')
                self.slam_process = None
                self.publish_status({'status': 'ok', 'message': 'Mapping stopped'})
            except Exception as e:
                self.get_logger().error(f'Failed to stop mapping: {e}')
                self.publish_status({'status': 'error', 'message': str(e)})

    def publish_status(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.status_publisher.publish(msg)

    def cleanup(self):
        """Ensure SLAM process is terminated on shutdown."""
        with self.slam_lock:
            if self.slam_process is not None:
                try:
                    self.slam_process.terminate()
                    self.slam_process.wait(timeout=2)
                except Exception:
                    try:
                        self.slam_process.kill()
                    except Exception:
                        pass


def main():
    rclpy.init()
    node = MappingLifecycleManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
