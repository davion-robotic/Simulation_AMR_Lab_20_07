#!/usr/bin/env python3
"""Comprehensive test runner for TopologyManagerNode and TopologyEdit."""

import os
import sys
import json
import yaml
from pathlib import Path
from std_msgs.msg import String

# Add src directories to PYTHONPATH
sys.path.insert(0, '/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_rmf')
sys.path.insert(0, '/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_common')

import rclpy
from mybot_rmf.topology_manager import TopologyManagerNode, RMF_MAPS_DIR

def run_test():
    rclpy.init()
    node = TopologyManagerNode()
    nav_path = RMF_MAPS_DIR / "map_lab20_topology.yaml"

    print("--- 1. Testing Add Vertex 'P5' ---")
    msg1 = String()
    msg1.data = json.dumps({
        "action": "add_vertex",
        "map_name": "map_lab20",
        "name": "P5",
        "x": 1.5,
        "y": 3.0,
        "is_parking_spot": True
    })
    node.on_handle_topology_request(msg1)

    with open(nav_path, 'r') as f:
        data = yaml.safe_load(f)
    verts = data['levels']['L1']['vertices']
    print(f"Vertices after add_vertex: {[v[2].get('name') for v in verts]}")
    assert any(v[2].get('name') == 'P5' for v in verts), "P5 should be added"

    print("\n--- 2. Testing Add Lane 'P4' -> 'P5' ---")
    msg2 = String()
    msg2.data = json.dumps({
        "action": "add_lane",
        "map_name": "map_lab20",
        "from": "P4",
        "to": "P5",
        "is_bidirectional": True
    })
    node.on_handle_topology_request(msg2)

    with open(nav_path, 'r') as f:
        data = yaml.safe_load(f)
    verts = data['levels']['L1']['vertices']
    lanes = data['levels']['L1']['lanes']
    p4_idx = next(i for i, v in enumerate(verts) if v[2].get('name') == 'P4')
    p5_idx = next(i for i, v in enumerate(verts) if v[2].get('name') == 'P5')
    print(f"Lanes count after add_lane: {len(lanes)} (P4: {p4_idx}, P5: {p5_idx})")
    assert any(l[0] == p4_idx and l[1] == p5_idx for l in lanes), f"Lane {p4_idx}->{p5_idx} should exist"
    assert any(l[0] == p5_idx and l[1] == p4_idx for l in lanes), f"Lane {p5_idx}->{p4_idx} should exist"

    print("\n--- 3. Testing Delete Lane 'P4' -> 'P5' with generic action='delete' ---")
    msg3 = String()
    msg3.data = json.dumps({
        "action": "delete",
        "map_name": "map_lab20",
        "from": "P4",
        "to": "P5"
    })
    node.on_handle_topology_request(msg3)

    with open(nav_path, 'r') as f:
        data = yaml.safe_load(f)
    lanes = data['levels']['L1']['lanes']
    assert not any(l[0] == p4_idx and l[1] == p5_idx for l in lanes), f"Lane {p4_idx}->{p5_idx} should be deleted"

    print("\n--- 4. Testing Delete Vertex 'P5' with generic action='remove' ---")
    msg4 = String()
    msg4.data = json.dumps({
        "action": "remove",
        "map_name": "map_lab20",
        "name": "P5"
    })
    node.on_handle_topology_request(msg4)

    with open(nav_path, 'r') as f:
        data = yaml.safe_load(f)
    verts = data['levels']['L1']['vertices']
    print(f"Vertices after delete_vertex: {[v[2].get('name') for v in verts]}")
    assert not any(v[2].get('name') == 'P5' for v in verts), "P5 should be deleted"

    print("\n--- 5. Testing Delete Lane with index 0 (0-falsy edge case) ---")
    # Add temporary lane 0 <-> 2
    node.handler.add_lane({"map_name": "map_lab20", "from": 0, "to": 2, "is_bidirectional": True})
    with open(nav_path, 'r') as f:
        lanes = yaml.safe_load(f)['levels']['L1']['lanes']
    assert any(l[0] == 0 and l[1] == 2 for l in lanes), "Lane 0->2 should exist"

    # Delete lane using integer 0
    msg5 = String()
    msg5.data = json.dumps({
        "action": "delete_lane",
        "map_name": "map_lab20",
        "from": 0,
        "to": 2
    })
    node.on_handle_topology_request(msg5)
    with open(nav_path, 'r') as f:
        lanes = yaml.safe_load(f)['levels']['L1']['lanes']
    assert not any(l[0] == 0 and l[1] == 2 for l in lanes), "Lane 0->2 should be deleted with from=0"
    print("✓ Successfully deleted lane with from=0 (0-falsy bug resolved)")

    print("\n--- 6. Testing Delete Lane with lane array [0, 2] and dict payload ---")
    node.handler.add_lane({"map_name": "map_lab20", "lane": [0, 2], "is_bidirectional": True})
    node.handler.delete_lane({"map_name": "map_lab20", "lane": [0, 2]})
    with open(nav_path, 'r') as f:
        lanes = yaml.safe_load(f)['levels']['L1']['lanes']
    assert not any(l[0] == 0 and l[1] == 2 for l in lanes), "Lane [0, 2] should be deleted"
    print("✓ Successfully deleted lane with array [0, 2]")

    print("\n--- 7. Testing Direct Action Topic Callback with plain string / unquoted payload ---")
    node.handler.add_vertex({"map_name": "map_lab20", "name": "P_TEMP", "x": 0.0, "y": 0.0})
    # Simulate direct message to /topology_manager/delete_vertex
    direct_msg = String()
    direct_msg.data = "P_TEMP"
    node._on_direct_action(direct_msg, 'delete_vertex')
    with open(nav_path, 'r') as f:
        verts = yaml.safe_load(f)['levels']['L1']['vertices']
    assert not any(v[2].get('name') == 'P_TEMP' for v in verts), "P_TEMP should be deleted via direct string"
    print("✓ Successfully handled direct string deletion on action topic")

    print("\n--- 8. Testing Goal Deletion Synchronization ---")
    node.handler.add_vertex({"map_name": "map_lab20", "name": "GOAL_TEST", "x": 1.0, "y": 1.0})
    goal_del_msg = String()
    goal_del_msg.data = json.dumps({"map_name": "map_lab20", "goal_id": "GOAL_TEST"})
    node._on_goal_deleted(goal_del_msg)
    with open(nav_path, 'r') as f:
        verts = yaml.safe_load(f)['levels']['L1']['vertices']
    assert not any(v[2].get('name') == 'GOAL_TEST' for v in verts), "GOAL_TEST should be synced and deleted"
    print("✓ Successfully synced goal deletion to Topology graph & broadcast")

    print("\n🎉 ALL 8 COMPREHENSIVE TOPOLOGY OPERATIONS PASSED 100% SUCCESSFUL VERIFICATION!")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_test()
