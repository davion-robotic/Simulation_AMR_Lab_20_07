# mybot_status

Package ROS 2 Python Trung Tâm Báo Cáo Trạng Thái Hệ Thống AMR (Unified Robot Status Hub).

Tự động thu thập, chuẩn hóa và xuất bản trạng thái toàn diện của xe qua topic:
- **`/robot_status_simple`** (Topic chuẩn hóa kết nối WebSocket Tablet/Web)
- **`/robot_status`** (Topic Master)

---

## 1. Cấu Trúc Gói Tin JSON `/robot_status_simple`

Gói tin **giữ nguyên 100% các trường cũ** (`status`, `current_goal`, `battery_percent`, `pose`) và **tích hợp thêm toàn bộ trường của `/task_controller/status`**:

```json
{
  "status": "waiting",
  "current_goal": "Bàn 1",
  "battery_percent": 96.5,
  "pose": {
    "x": 2.35,
    "y": 1.48
  },
  "task_state": "WAITING",
  "dwell_time_remaining": 28,
  "mission_time_remaining": 7120,
  "current_waypoint_index": 0,
  "total_waypoints": 1,
  "is_paused": false,
  "statistics": {
    "completed_count": 1,
    "skipped_count": 0,
    "elapsed_time_sec": 45
  },
  "task": {
    "state": "WAITING",
    "sub_state": "DWELLING",
    "mission_type": "DELIVERY",
    "current_goal_id": "P1",
    "current_goal_name": "Bàn 1",
    "dwell_time_remaining": 28,
    "mission_time_remaining": 7120,
    "is_paused": false
  },
  "message": "Giao hàng tại Bàn 1 (28s còn lại)"
}
```

---

## 2. Nguồn Thu Thập Dữ Liệu Tự Động

- **Tọa độ (`pose`)**: Lắng nghe `/amcl_pose` (hoặc `/odom`).
- **Mức pin (`battery_percent`)**: Lắng nghe `/battery_status` hoặc `/battery_percent`.
- **Nghiệp vụ giao hàng / tuần tra (`task_state`, `dwell_time_remaining`, `task`)**: Lắng nghe trực tiếp từ `/task_controller/status`.
- **Trạng thái chuyển động**: Lắng nghe `/cmd_vel` và Nav2 Action Server.

---

## 3. Khởi Chạy

Node được khởi chạy tự động trong:
```bash
ros2 launch mybot_navigation2_tablet tablet_bridge.launch.py
```
*(hoặc chạy riêng: `ros2 run mybot_status robot_status_publisher`)*

