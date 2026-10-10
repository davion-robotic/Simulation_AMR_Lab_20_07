#!/usr/bin/env python3
"""Centralized topic and service names for AMR Robot."""

# Mapping Tool Topics
MAPPING_SCAN_TOPIC = '/mapping_tool/scan'
MAPPING_SCAN_STATUS_TOPIC = '/mapping_tool/scan_status'
MAPPING_SAVE_MAP_TOPIC = '/mapping_tool/save_map'
MAPPING_SAVE_MAP_RESULT_TOPIC = '/mapping_tool/save_map_result'
MAPPING_SAVE_EDITED_MAP_TOPIC = '/mapping_tool/save_edited_map'
MAPPING_SAVE_EDITED_MAP_RESULT_TOPIC = '/mapping_tool/save_edited_map_result'

# Map Manager Topics
MAP_MANAGER_GET_AVAILABLE_MAPS_REQ_TOPIC = '/map_manager/get_available_maps_req'
MAP_MANAGER_AVAILABLE_MAPS_TOPIC = '/map_manager/available_maps'
MAP_MANAGER_LOAD_MAP_TOPIC = '/map_manager/load_map'
MAP_MANAGER_LOAD_MAP_RESULT_TOPIC = '/map_manager/load_map_result'

# Nav2 Services & Topics
NAV2_MAP_SERVER_LOAD_MAP_SERVICE = '/map_server/load_map'

# Robot Status Topics
ROBOT_STATUS_SIMPLE_TOPIC = '/robot_status_simple'
AMCL_POSE_TOPIC = '/amcl_pose'
ODOM_TOPIC = '/odom'
CMD_VEL_TOPIC = '/cmd_vel'

# Goal Manager Topics
GOAL_CREATE_TOPIC = '/goal_manager/create_goal'
GOAL_UPDATE_TOPIC = '/goal_manager/update_goal'
GOAL_DELETE_TOPIC = '/goal_manager/delete_goal'
GOAL_GET_LIST_REQ_TOPIC = '/goal_manager/get_goals_req'
GOAL_LIST_TOPIC = '/goal_manager/goals_list'
GOAL_AVAILABLE_GOALS_TOPIC = '/goal_manager/available_goals'
GOAL_RESULT_TOPIC = '/goal_manager/result'
GOAL_GO_TO_TOPIC = '/goal_manager/go_to_goal'
GOAL_CANCEL_TOPIC = '/goal_manager/cancel_goal'

# Localization Topics
LOCALIZATION_START_TOPIC = '/localization/start'
LOCALIZATION_STOP_TOPIC = '/localization/stop'
LOCALIZATION_INITIAL_POSE_TOPIC = '/localization/initial_pose'
LOCALIZATION_RESET_POSE_TOPIC = '/localization/reset_pose'
LOCALIZATION_STATUS_TOPIC = '/localization/status'
NAV2_INITIAL_POSE_TOPIC = '/initialpose'

# Topology Manager Topics
TOPOLOGY_SAVE_TOPIC = '/topology_manager/save_topology'
TOPOLOGY_RESULT_TOPIC = '/topology_manager/result'
TOPOLOGY_GET_REQ_TOPIC = '/topology_manager/get_topology_req'
TOPOLOGY_INFO_TOPIC = '/topology_manager/topology_info'
TOPOLOGY_AVAILABLE_TOPIC = '/topology_manager/available_topology'
TOPOLOGY_RELOAD_TOPIC = '/topology_manager/reload'



