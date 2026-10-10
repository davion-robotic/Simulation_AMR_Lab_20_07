#!/bin/bash
echo "Đang dọn dẹp toàn bộ tiến trình ROS 2, Nav2 và Gazebo cũ..."

# Tắt các tiến trình launch cũ
pkill -9 -f "ros2 launch mybot" 2>/dev/null
pkill -9 -f "gazebo.launch.py" 2>/dev/null

# Tắt các tiến trình của Nav2
pkill -9 -f nav2_
pkill -9 -f controller_server
pkill -9 -f planner_server
pkill -9 -f bt_navigator
pkill -9 -f map_server
pkill -9 -f localization_launch
pkill -9 -f amcl
pkill -9 -f lifecycle_manager

# Tắt các ứng dụng đồ họa và mô phỏng
pkill -9 -f rviz2
pkill -9 -f gazebo
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f robot_state_publisher
pkill -9 -f rosbridge_websocket

# Tắt tất cả các node Python bị kẹt lại
pkill -9 -f mybot
pkill -9 -f initial_pose
pkill -9 -f load_map
pkill -9 -f get_goals
pkill -9 -f delete_goal
pkill -9 -f create_goal
pkill -9 -f goal_manager

# Dọn dẹp rác bộ nhớ chia sẻ của FastDDS
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

echo "==============================================="
echo "✅ Dọn dẹp hoàn tất! Hệ thống đã sạch sẽ."
echo "Bây giờ bạn có thể mở các Terminal mới để chạy lại."
echo "==============================================="
