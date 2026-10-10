#!/usr/bin/env python3
"""
topology_sniffer.py - Monitor tat ca payload App gui len cac topic Topology.
Chay song song voi he thong de debug xem App dang gui gi.

Su dung:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/mybot_rmf/scripts/topology_sniffer.py

    # Xem ca response topic (topology_info, available_topology - ban 1s/lan):
    python3 src/mybot_rmf/scripts/topology_sniffer.py --all
"""

import json
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

# =========================================================
# Topic LENH tu App gui vao (add/edit/delete vertex & lane)
# =========================================================
COMMAND_TOPICS = [
    # Topic chinh - nhan lenh thao tac topology
    '/topology_manager/save_topology',
    '/map_manager/save_topology',
    '/topology_manager/edit_topology',
    '/map_manager/edit_topology',

    # Add vertex
    '/topology_manager/add_vertex',
    '/topology_manager/create_vertex',
    '/topology_manager/add_node',
    '/topology_manager/add_point',
    '/topology_manager/add_waypoint',
    '/map_manager/add_vertex',
    '/map_manager/add_node',
    '/map_manager/add_point',
    '/map_manager/add_waypoint',

    # Edit vertex
    '/topology_manager/edit_vertex',
    '/topology_manager/update_vertex',
    '/map_manager/edit_vertex',
    '/map_manager/update_vertex',

    # Delete vertex
    '/topology_manager/delete_vertex',
    '/topology_manager/remove_vertex',
    '/map_manager/delete_vertex',
    '/map_manager/remove_vertex',

    # Add lane
    '/topology_manager/add_lane',
    '/topology_manager/create_lane',
    '/topology_manager/add_edge',
    '/map_manager/add_lane',
    '/map_manager/create_lane',
    '/map_manager/add_edge',

    # Edit lane
    '/topology_manager/edit_lane',
    '/topology_manager/update_lane',
    '/map_manager/edit_lane',
    '/map_manager/update_lane',

    # Delete lane
    '/topology_manager/delete_lane',
    '/topology_manager/remove_lane',
    '/map_manager/delete_lane',
    '/map_manager/remove_lane',

    # Result (response 1 lan sau moi lenh)
    '/topology_manager/topology_result',
    '/topology_manager/topology_reload',
]

# Topic OUTPUT - ban 1s/lan (se an i tru khi chay --all)
PERIODIC_TOPICS = [
    '/topology_manager/topology_info',
    '/topology_manager/available_topology',
]

# Xet xem co flag --all khong
SHOW_ALL = '--all' in sys.argv
WATCH_TOPICS = COMMAND_TOPICS + (PERIODIC_TOPICS if SHOW_ALL else [])

# Mau terminal ANSI
RESET   = '\033[0m'
BOLD    = '\033[1m'
GREEN   = '\033[92m'
YELLOW  = '\033[93m'
RED     = '\033[91m'
CYAN    = '\033[96m'
BLUE    = '\033[94m'
GRAY    = '\033[90m'
MAGENTA = '\033[95m'

TOPIC_COLORS = {
    'add':       GREEN,
    'create':    GREEN,
    'insert':    GREEN,
    'edit':      YELLOW,
    'update':    YELLOW,
    'modify':    YELLOW,
    'delete':    RED,
    'remove':    RED,
    'result':    CYAN,
    'info':      CYAN,
    'available': BLUE,
    'reload':    MAGENTA,
    'save':      YELLOW,
}


def topic_color(topic: str) -> str:
    for key, color in TOPIC_COLORS.items():
        if key in topic:
            return color
    return RESET


def format_payload(data_str: str) -> str:
    """Parse va format JSON payload dep, detect action, from/to, name..."""
    try:
        payload = json.loads(data_str)
        if not isinstance(payload, dict):
            return f"  [!] Non-dict payload: {repr(data_str[:200])}"

        lines = []

        # Detect action
        action = ""
        for k in ["action", "command", "cmd", "op", "type", "operation"]:
            if k in payload and payload[k]:
                action = str(payload[k])
                break

        if action:
            lines.append(f"  [ACTION]  {BOLD}{action}{RESET}")

        # Map name
        if "map_name" in payload:
            lines.append(f"  [MAP]     {payload['map_name']}")

        # Vertex info
        for k in ["name", "vertex_name", "goal_id", "vertex_id", "point_name"]:
            if k in payload and payload[k]:
                lines.append(f"  [VERTEX]  {k} = {payload[k]}")
                break

        if "old_name" in payload:
            lines.append(f"  [RENAME]  old_name = {payload['old_name']}")
        if "new_name" in payload:
            lines.append(f"  [RENAME]  new_name = {payload['new_name']}")

        if "x" in payload or "y" in payload:
            x = payload.get("x", "?")
            y = payload.get("y", "?")
            lines.append(f"  [POS]     x={x}, y={y}")

        # Lane info
        for k in ["from", "start", "from_vertex", "start_vertex", "u", "source"]:
            if k in payload and payload[k] is not None:
                lines.append(f"  [FROM]    {payload[k]}")
                break

        for k in ["to", "end", "to_vertex", "end_vertex", "v"]:
            if k in payload and payload[k] is not None:
                lines.append(f"  [TO]      {payload[k]}")
                break

        if "lane" in payload:
            lines.append(f"  [LANE]    {payload['lane']}")

        if "is_bidirectional" in payload or "bidirectional" in payload:
            bidi = payload.get("is_bidirectional", payload.get("bidirectional"))
            lines.append(f"  [BIDI]    {bidi}")

        if "speed_limit" in payload:
            lines.append(f"  [SPEED]   {payload['speed_limit']} m/s")

        # Props
        prop_keys = ["is_charger", "is_parking_spot", "is_pickup", "is_destination", "is_boost"]
        props_set = {k: payload[k] for k in prop_keys if k in payload}
        if props_set:
            lines.append(f"  [PROPS]   {props_set}")

        # Response status
        if "status" in payload:
            status = payload["status"]
            icon = "[OK]" if status == "ok" else "[ERR]"
            lines.append(f"  {icon}    status = {status}")
        if "message" in payload:
            lines.append(f"  [MSG]     {payload['message']}")

        # Extra remaining keys
        shown = {
            "action", "command", "cmd", "op", "type", "operation", "map_name",
            "name", "vertex_name", "goal_id", "vertex_id", "point_name",
            "old_name", "new_name", "x", "y",
            "from", "start", "from_vertex", "start_vertex", "u", "source",
            "to", "end", "to_vertex", "end_vertex", "v", "target",
            "lane", "is_bidirectional", "bidirectional", "speed_limit",
            "is_charger", "is_parking_spot", "is_pickup", "is_destination", "is_boost",
            "status", "message"
        }
        extra = {k: payload[k] for k in payload if k not in shown}
        if extra:
            lines.append(f"  [EXTRA]   {json.dumps(extra, ensure_ascii=False)[:300]}")

        return "\n".join(lines) if lines else f"  {json.dumps(payload, ensure_ascii=False)[:400]}"

    except json.JSONDecodeError:
        return f"  [RAW]  {repr(data_str[:300])}"
    except Exception as e:
        return f"  [PARSE ERR] {e} | raw: {repr(data_str[:200])}"


class TopologySnifferNode(Node):
    def __init__(self):
        super().__init__('topology_sniffer')
        self._msg_count = 0

        mode = "TAT CA topics (--all)" if SHOW_ALL else "CHI LENH tu App (mac dinh)"
        print(f"\n{BOLD}{CYAN}{'=' * 65}{RESET}")
        print(f"{BOLD}{CYAN}  TOPOLOGY SNIFFER - Lang nghe {len(WATCH_TOPICS)} topics{RESET}")
        print(f"{BOLD}{YELLOW}  Mode: {mode}{RESET}")
        print(f"{BOLD}{CYAN}{'=' * 65}{RESET}")
        print(f"{GRAY}  Nhan Ctrl+C de dung{RESET}")
        if not SHOW_ALL:
            print(f"{GRAY}  (Them --all de xem ca topology_info/available_topology){RESET}\n")
        else:
            print()

        self._subs = []
        for topic in WATCH_TOPICS:
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, t=topic: self._on_msg(msg, t),
                10
            )
            self._subs.append(sub)
            print(f"  {GRAY}>> Watching: {topic}{RESET}")

        print(f"\n{BOLD}{'─' * 65}{RESET}")
        print(f"{BOLD}  Cho message tu App...{RESET}")
        print(f"{BOLD}{'─' * 65}{RESET}\n")

    def _on_msg(self, msg: String, topic: str):
        self._msg_count += 1
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = topic_color(topic)

        sep = '=' * 65
        print(f"\n{color}{BOLD}{sep}{RESET}")
        print(f"{color}{BOLD}  #{self._msg_count:04d} [{now}]{RESET}")
        print(f"{color}{BOLD}  TOPIC: {topic}{RESET}")
        print(f"{color}{'─' * 65}{RESET}")
        print(format_payload(msg.data))
        print(f"{GRAY}  raw: {repr(msg.data[:250])}{RESET}")


def main():
    rclpy.init()
    node = TopologySnifferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n\n{YELLOW}Sniffer dung. Tong messages da bat: {node._msg_count}{RESET}\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
