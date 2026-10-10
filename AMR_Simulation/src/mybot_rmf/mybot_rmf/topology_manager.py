#!/usr/bin/env python3
"""topology_manager.py: ROS 2 Node Adapter quản lý giao tiếp Topic & Service cho Topology Manager.

Tất cả logic lõi xử lý dữ liệu đồ thị Topology được ủy quyền cho TopologyEdit.
"""

import json
import re
from typing import Any, Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from mybot_common.command_utils import extract_map_name, parse_payload_dict, sanitize_map_name
from mybot_common.topic_names import (
    GOAL_DELETE_TOPIC,
    MAP_MANAGER_LOAD_MAP_TOPIC,
    ROBOT_STATUS_SIMPLE_TOPIC,
    TOPOLOGY_AVAILABLE_TOPIC,
    TOPOLOGY_GET_REQ_TOPIC,
    TOPOLOGY_INFO_TOPIC,
    TOPOLOGY_RELOAD_TOPIC,
    TOPOLOGY_RESULT_TOPIC,
    TOPOLOGY_SAVE_TOPIC,
)
from mybot_rmf.topology_edit import RMF_MAPS_DIR, TopologyEdit


class TopologyManagerNode(Node):
    """ROS 2 Node Adapter đảm nhận giao tiếp ROS 2 Topic & Service cho Topology System."""

    def __init__(self):
        super().__init__('topology_manager')

        self.handler = TopologyEdit(maps_dir=RMF_MAPS_DIR)

        self.active_map_name = 'map_lab20'
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_status = 'idle'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.sub_robot_status = self.create_subscription(String, ROBOT_STATUS_SIMPLE_TOPIC, self._robot_status_cb, 10)

        self.sub_load_map = self.create_subscription(String, MAP_MANAGER_LOAD_MAP_TOPIC, self.on_map_loaded, 10)

        self.sub_save = self.create_subscription(String, TOPOLOGY_SAVE_TOPIC, self.on_handle_topology_request, 10)
        self.sub_save_map = self.create_subscription(String, '/map_manager/save_topology', self.on_handle_topology_request, 10)

        vertex_del_topics = [
            '/topology_manager/delete_vertex', '/topology_manager/remove_vertex', '/topology_manager/del_vertex',
            '/topology_manager/delete_node', '/topology_manager/remove_node', '/topology_manager/delete_point',
            '/topology_manager/remove_point', '/topology_manager/delete_waypoint', '/topology_manager/remove_waypoint',
            '/map_manager/delete_vertex', '/map_manager/remove_vertex', '/map_manager/del_vertex',
            '/map_manager/delete_node', '/map_manager/remove_node', '/map_manager/delete_point',
            '/map_manager/remove_point', '/map_manager/delete_waypoint', '/map_manager/remove_waypoint'
        ]
        for top in vertex_del_topics:
            self.create_subscription(String, top, lambda msg, act='delete_vertex': self._on_direct_action(msg, act), 10)

        lane_del_topics = [
            '/topology_manager/delete_lane', '/topology_manager/remove_lane', '/topology_manager/del_lane',
            '/topology_manager/delete_edge', '/topology_manager/remove_edge', '/topology_manager/delete_link',
            '/topology_manager/remove_link', '/topology_manager/delete_route', '/topology_manager/delete_segment',
            '/map_manager/delete_lane', '/map_manager/remove_lane', '/map_manager/del_lane',
            '/map_manager/delete_edge', '/map_manager/remove_edge', '/map_manager/delete_link',
            '/map_manager/remove_link', '/map_manager/delete_route', '/map_manager/delete_segment'
        ]
        for top in lane_del_topics:
            self.create_subscription(String, top, lambda msg, act='delete_lane': self._on_direct_action(msg, act), 10)

        vertex_add_topics = [
            '/topology_manager/add_vertex', '/topology_manager/create_vertex', '/topology_manager/add_node',
            '/topology_manager/add_point', '/topology_manager/add_waypoint', '/map_manager/add_vertex',
            '/map_manager/create_vertex', '/map_manager/add_node', '/map_manager/add_point', '/map_manager/add_waypoint'
        ]
        for top in vertex_add_topics:
            self.create_subscription(String, top, lambda msg, act='add_vertex': self._on_direct_action(msg, act), 10)

        vertex_edit_topics = [
            '/topology_manager/edit_vertex', '/topology_manager/update_vertex', '/topology_manager/modify_vertex',
            '/topology_manager/edit_point', '/topology_manager/update_point', '/topology_manager/modify_point',
            '/topology_manager/edit_node', '/topology_manager/update_node', '/topology_manager/modify_node',
            '/topology_manager/edit_waypoint', '/topology_manager/update_waypoint', '/topology_manager/modify_waypoint',
            '/map_manager/edit_vertex', '/map_manager/update_vertex', '/map_manager/modify_vertex',
            '/map_manager/edit_point', '/map_manager/update_point', '/map_manager/modify_point',
            '/map_manager/edit_node', '/map_manager/update_node', '/map_manager/modify_node',
            '/map_manager/edit_waypoint', '/map_manager/update_waypoint', '/map_manager/modify_waypoint'
        ]
        for top in vertex_edit_topics:
            self.create_subscription(String, top, lambda msg, act='edit_vertex': self._on_direct_action(msg, act), 10)

        lane_add_topics = [
            '/topology_manager/add_lane', '/topology_manager/create_lane', '/topology_manager/add_edge',
            '/topology_manager/create_edge', '/map_manager/add_lane', '/map_manager/create_lane',
            '/map_manager/add_edge', '/map_manager/create_edge'
        ]
        for top in lane_add_topics:
            self.create_subscription(String, top, lambda msg, act='add_lane': self._on_direct_action(msg, act), 10)

        lane_edit_topics = [
            '/topology_manager/edit_lane', '/topology_manager/update_lane', '/topology_manager/modify_lane',
            '/topology_manager/edit_edge', '/topology_manager/update_edge', '/topology_manager/modify_edge',
            '/map_manager/edit_lane', '/map_manager/update_lane', '/map_manager/modify_lane',
            '/map_manager/edit_edge', '/map_manager/update_edge', '/map_manager/modify_edge'
        ]
        for top in lane_edit_topics:
            self.create_subscription(String, top, lambda msg, act='edit_lane': self._on_direct_action(msg, act), 10)

        self.sub_edit = self.create_subscription(String, '/topology_manager/edit_topology', self.on_handle_topology_request, 10)
        self.sub_map_edit = self.create_subscription(String, '/map_manager/edit_topology', self.on_handle_topology_request, 10)

        self.sub_goal_del = self.create_subscription(String, GOAL_DELETE_TOPIC, self._on_goal_deleted, 10)
        self.sub_get_req = self.create_subscription(String, TOPOLOGY_GET_REQ_TOPIC, self.on_get_topology, 10)

        self.srv_save_map = self.create_service(Trigger, '/map_manager/save_topology', self.on_save_topology_service)
        self.srv_save_top = self.create_service(Trigger, '/topology_manager/save_topology', self.on_save_topology_service)

        self.pub_result = self.create_publisher(String, TOPOLOGY_RESULT_TOPIC, 10)
        self.pub_info = self.create_publisher(String, TOPOLOGY_INFO_TOPIC, 10)
        self.pub_available_topology = self.create_publisher(String, TOPOLOGY_AVAILABLE_TOPIC, 10)
        self.pub_reload = self.create_publisher(String, TOPOLOGY_RELOAD_TOPIC, 10)

        self._cached_payload_str = ''
        self.publish_available_topology(self.active_map_name)

        self.timer = self.create_timer(1.0, self._periodic_publish)
        self.get_logger().info('TopologyManagerNode (ROS 2 Adapter) Ready!')

    def _periodic_publish(self):
        """Bắn định kỳ dữ liệu Topology mỗi 1.0s để App/Tablet không bị lỡ dữ liệu khi kết nối WebSocket."""
        if self._cached_payload_str:
            out = String()
            out.data = self._cached_payload_str
            self.pub_available_topology.publish(out)
            self.pub_info.publish(out)

    def _on_direct_action(self, msg: String, default_action: str):
        try:
            payload = parse_payload_dict(msg.data)
            if not isinstance(payload, dict):
                payload = {'data': payload}
            if 'action' not in payload and 'command' not in payload and 'cmd' not in payload and 'op' not in payload:
                payload['action'] = default_action
            out = String()
            out.data = json.dumps(payload)
            self.on_handle_topology_request(out)
        except Exception as e:
            self.get_logger().error(f"❌ Error in direct action '{default_action}': {e}")

    def _on_goal_deleted(self, msg: String):
        try:
            payload = parse_payload_dict(msg.data)
            map_name = payload.get('map_name') or self.active_map_name
            goal_id = payload.get('goal_id') or payload.get('name')
            if goal_id:
                try:
                    self.handler.delete_vertex({'map_name': map_name, 'name': goal_id})
                except Exception:
                    pass
                self.publish_available_topology(map_name)
                self.broadcast_reload(f"Synced goal deletion for '{goal_id}'")
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)

    def _robot_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.robot_status = str(data.get('status', 'idle')).lower()
        except Exception:
            pass

    def update_robot_pose_from_tf(self):
        try:
            if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)):
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                p = t.transform.translation
                self.robot_x = float(p.x)
                self.robot_y = float(p.y)
        except Exception:
            pass

    def publish_result(self, result_dict: dict):
        out = String()
        out.data = json.dumps(result_dict)
        self.pub_result.publish(out)

    def publish_info(self, info_dict: dict):
        out = String()
        out.data = json.dumps(info_dict)
        self.pub_info.publish(out)

    def publish_available_topology(self, map_name: str, full_data: dict = None):
        """Xuất bản dữ liệu Topology chuẩn Open-RMF cho App/Tablet."""
        try:
            if full_data is None:
                full_data, _, _, _, _ = self.handler.load_graph(map_name, self.active_map_name)

            levels = full_data.get('levels', {})
            norm_levels = {}
            for lvl_name, lvl_data in levels.items():
                norm_verts = []
                for i, v in enumerate(lvl_data.get('vertices', [])):
                    x = float(v[0])
                    y = float(v[1])
                    p = v[2] if len(v) > 2 and isinstance(v[2], dict) else {}
                    norm_p = {
                        'name': str(p.get('name') or f'P{i+1}').strip(),
                        'is_charger': bool(p.get('is_charger', False)),
                        'is_parking_spot': bool(p.get('is_parking_spot', False)),
                        'is_pickup': bool(p.get('is_pickup', False)),
                        'is_destination': bool(p.get('is_destination', False)),
                        'is_boost': bool(p.get('is_boost', False)),
                        'yaw': float(p.get('yaw', 0.0))
                    }
                    norm_verts.append([x, y, norm_p])

                norm_lanes = []
                for lane in lvl_data.get('lanes', []):
                    u = int(lane[0])
                    v_idx = int(lane[1])
                    lp = lane[2] if len(lane) > 2 and isinstance(lane[2], dict) else {}
                    speed = float(lp.get('speed_limit', 0.8))
                    lane_dict = {'speed_limit': speed}
                    if 'is_bidirectional' in lp:
                        lane_dict['is_bidirectional'] = bool(lp['is_bidirectional'])
                    norm_lanes.append([u, v_idx, lane_dict])

                norm_levels[lvl_name] = {
                    'vertices': norm_verts,
                    'lanes': norm_lanes
                }

            payload = {
                'map_name': map_name,
                'building_name': str(full_data.get('building_name') or 'topology'),
                'levels': norm_levels,
            }
            if 'lifts' in full_data and full_data['lifts']:
                payload['lifts'] = full_data['lifts']
            if 'doors' in full_data and full_data['doors']:
                payload['doors'] = full_data['doors']

            payload_str = json.dumps(payload, ensure_ascii=False)
            if map_name == self.active_map_name:
                self._cached_payload_str = payload_str

            out = String()
            out.data = payload_str
            self.pub_available_topology.publish(out)
            self.pub_info.publish(out)
            self.get_logger().info(f"📤 Published available topology for '{map_name}' to {TOPOLOGY_AVAILABLE_TOPIC}")
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing available topology for '{map_name}': {e}")

    def broadcast_reload(self, message_str: str = 'Topology map updated'):
        out = String()
        out.data = json.dumps({
            'action': 'reload',
            'map_name': self.active_map_name,
            'message': message_str,
        })
        self.pub_reload.publish(out)

    def on_map_loaded(self, msg: String):
        raw_name = extract_map_name(msg.data)
        map_name = sanitize_map_name(raw_name)
        if not map_name:
            return

        self.get_logger().info(f"🔄 Map switched to '{map_name}'. Syncing topology for '{map_name}'...")
        self.active_map_name = map_name

        full_data, _, _, _, _ = self.handler.load_graph(map_name, self.active_map_name)
        self.handler.save_atomic(full_data, map_name=map_name, active_map=self.active_map_name)
        self.broadcast_reload(f"Synced topology for map '{map_name}'")
        self.publish_available_topology(map_name, full_data)

    def on_handle_topology_request(self, msg: String):
        try:
            payload = parse_payload_dict(msg.data)
            if not payload:
                self.publish_result({'status': 'error', 'message': 'Empty or invalid JSON payload'})
                return

            self.update_robot_pose_from_tf()

            action = ''
            for act_k in ['action', 'command', 'cmd', 'op', 'type', 'operation', 'method']:
                if act_k in payload and payload[act_k]:
                    action = str(payload[act_k]).strip()
                    break

            clean_action = re.sub(r'[^a-zA-Z0-9]', '', action).lower()
            target_map = payload.get('map_name') or payload.get('mapName') or payload.get('map') or payload.get('active_map') or self.active_map_name

            vertex_add_set = {
                'addvertex', 'createvertex', 'insertvertex', 'addnode', 'createnode', 'insertnode',
                'addpoint', 'createpoint', 'insertpoint', 'addwaypoint', 'createwaypoint', 'insertwaypoint',
                'newvertex', 'newnode', 'newpoint', 'newwaypoint'
            }
            vertex_edit_set = {
                'editvertex', 'updatevertex', 'modifyvertex', 'editnode', 'updatenode', 'modifynode',
                'editpoint', 'updatepoint', 'modifypoint', 'editwaypoint', 'updatewaypoint', 'modifywaypoint',
                'changevertex', 'changenode', 'changepoint'
            }
            vertex_del_set = {
                'deletevertex', 'removevertex', 'delvertex', 'deletenode', 'removenode', 'delnode',
                'deletepoint', 'removepoint', 'delpoint', 'deletewaypoint', 'removewaypoint', 'delwaypoint',
                'rmvertex', 'rmnode', 'rmpoint', 'rmwaypoint'
            }
            lane_add_set = {
                'addlane', 'createlane', 'insertlane', 'addedge', 'createedge', 'insertedge',
                'addlink', 'createlink', 'insertlink', 'addroute', 'createroute', 'addsegment', 'createsegment',
                'addpath', 'createpath', 'newlane', 'newedge', 'newlink', 'newroute'
            }
            lane_edit_set = {
                'editlane', 'updatelane', 'modifylane', 'editedge', 'updateedge', 'modifyedge',
                'editlink', 'updatelink', 'modifylink', 'editroute', 'updateroute', 'modifyroute',
                'editsegment', 'updatesegment', 'modifysegment', 'editpath', 'updatepath'
            }
            lane_del_set = {
                'deletelane', 'removelane', 'dellane', 'deleteedge', 'removeedge', 'deledge',
                'deletelink', 'removelink', 'dellink', 'deleteroute', 'removeroute', 'delroute',
                'deletesegment', 'removesegment', 'delsegment', 'deletepath', 'removepath', 'delpath',
                'rmlane', 'rmedge', 'rmlink', 'rmroute', 'rmsegment', 'rmpath'
            }

            def _check_lane_indicators(p):
                entity_type = str(p.get('entity') or p.get('target_type') or p.get('item_type') or '').strip().lower()
                if entity_type in ['lane', 'edge', 'link', 'route', 'segment', 'path']:
                    return True
                if any(k in p for k in ['lane', 'edge', 'link', 'route', 'segment', 'lane_index', 'laneIndex', 'lane_id', 'laneId', 'edge_id', 'edgeId']):
                    return True
                has_from = any(k in p for k in [
                    'from', 'start', 'source', 'start_vertex', 'from_vertex', 'fromVertex', 'startVertex',
                    'from_node', 'start_node', 'fromNode', 'startNode', 'from_point', 'start_point',
                    'fromPoint', 'startPoint', 'source_node', 'sourceNode', 'node1', 'node_1', 'point1', 'point_1', 'u', 'v1'
                ])
                has_to = any(k in p for k in [
                    'to', 'end', 'target', 'end_vertex', 'to_vertex', 'toVertex', 'endVertex',
                    'to_node', 'end_node', 'toNode', 'endNode', 'to_point', 'end_point',
                    'toPoint', 'endPoint', 'target_node', 'targetNode', 'node2', 'node_2', 'point2', 'point_2', 'v', 'v2'
                ])
                return has_from and has_to

            if clean_action in ['delete', 'remove', 'del', 'rm']:
                if _check_lane_indicators(payload):
                    clean_action = 'deletelane'
                else:
                    clean_action = 'deletevertex'
            elif clean_action in ['add', 'create', 'insert', 'new']:
                if _check_lane_indicators(payload):
                    clean_action = 'addlane'
                else:
                    clean_action = 'addvertex'
            elif clean_action in ['edit', 'update', 'modify', 'change']:
                if _check_lane_indicators(payload):
                    clean_action = 'editlane'
                else:
                    clean_action = 'editvertex'

            if clean_action in vertex_add_set:
                res = self.handler.add_vertex(payload, active_map=self.active_map_name, is_edit=False)
                target_map = res.get('map_name') or target_map
                self.active_map_name = target_map
            elif clean_action in vertex_edit_set:
                res = self.handler.edit_vertex(payload, active_map=self.active_map_name)
                target_map = res.get('map_name') or target_map
                self.active_map_name = target_map
            elif clean_action in vertex_del_set:
                res = self.handler.delete_vertex(payload, active_map=self.active_map_name, robot_x=self.robot_x, robot_y=self.robot_y, robot_status=self.robot_status)
                target_map = res.get('map_name') or target_map
                self.active_map_name = target_map
            elif clean_action in lane_add_set:
                res = self.handler.add_lane(payload, active_map=self.active_map_name)
                target_map = res.get('map_name') or target_map
                self.active_map_name = target_map
            elif clean_action in lane_edit_set:
                res = self.handler.edit_lane(payload, active_map=self.active_map_name)
                target_map = res.get('map_name') or target_map
                self.active_map_name = target_map
            elif clean_action in lane_del_set:
                res = self.handler.delete_lane(payload, active_map=self.active_map_name, robot_x=self.robot_x, robot_y=self.robot_y, robot_status=self.robot_status)
                target_map = res.get('map_name') or target_map
                self.active_map_name = target_map
            else:
                res = self.handler.full_save(payload, active_map=self.active_map_name)
                target_map = res.get('map_name') or target_map
                self.active_map_name = target_map

            self.publish_result(res)
            self.publish_info(res)
            self.publish_available_topology(target_map)
            self.broadcast_reload(f"Executed topology action '{action}'")

        except Exception as e:
            self.get_logger().error(f"\n❌ [TOPOLOGY] ⚠️ ACTION FAILED: {e}")
            self.publish_result({'status': 'error', 'action': 'topology_op', 'message': str(e)})

    def on_save_topology_service(self, request, response):
        try:
            self.get_logger().info(f"📥 [UI] Received Service save_topology request for map '{self.active_map_name}'")
            full_data, map_name, _, v, l = self.handler.load_graph(self.active_map_name, self.active_map_name)
            valid, err = self.handler.validate(v, l)
            if not valid:
                response.success = False
                response.message = f"Validation failed: {err}"
                self.get_logger().error(f"❌ Topology validation failed: {err}")
                return response

            success, msg = self.handler.save_atomic(full_data, map_name=map_name, active_map=self.active_map_name)
            response.success = success
            response.message = f"Saved: {msg}"
            self.get_logger().info(f"💾 [UI] Successfully saved topology via Service for map '{map_name}' -> {msg}")
            self.publish_available_topology(map_name, full_data)
            self.broadcast_reload('Saved topology via service')
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"❌ Error in save_topology service: {e}")
        return response

    def on_get_topology(self, msg: String):
        try:
            payload = parse_payload_dict(msg.data)
            target_map = payload.get('map_name') or self.active_map_name
            full_data, _, _, _, _ = self.handler.load_graph(target_map, self.active_map_name)
            self.publish_available_topology(target_map, full_data)
            self.publish_info({
                'status': 'ok',
                'action': 'get_topology',
                'map_name': target_map,
                'topology': full_data,
            })
        except Exception as e:
            self.publish_result({'status': 'error', 'action': 'get_topology', 'message': str(e)})


def main():
    rclpy.init()
    node = TopologyManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
