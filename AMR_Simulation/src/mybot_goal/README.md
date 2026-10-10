# Hướng Dẫn Sử Dụng Package `mybot_goal`

Package **`mybot_goal`** cung cấp giải pháp quản lý các điểm đích (Waypoints / Goals) độc lập cho từng bản đồ trên hệ thống AMR Robot.

---

## 1. Kiến trúc Package

Package được thiết kế theo nguyên tắc đơn nhiệm (Single Responsibility), mỗi chức năng chính được tách thành một node ROS 2 độc lập:

```text
mybot_goal/
├── mybot_goal/
│   ├── __init__.py
│   ├── goal_storage.py    # Xử lý đọc/ghi/xóa dữ liệu file JSON
│   ├── create_goal.py     # Node xử lý tạo Goal mới
│   ├── update_goal.py     # Node xử lý cập nhật Goal
│   ├── delete_goal.py     # Node xử lý xóa Goal
│   ├── get_goals.py       # Node xử lý lấy danh sách Goal
│   └── goal_manager.py    # Node tổng hợp (nếu muốn chạy chung)
├── package.xml
├── setup.py
└── README.md
```

---

## 2. Các Topic Giao Tiếp

Tất cả các topic sử dụng kiểu dữ liệu `std_msgs/msg/String` (chuỗi JSON).

| Chức năng | Topic Nhận Lệnh | Topic Trả Danh Sách / Kết Quả |
| :--- | :--- | :--- |
| **Tạo Goal** | `/goal_manager/create_goal` | `/goal_manager/result`, `/goal_manager/goals_list` |
| **Cập nhật Goal** | `/goal_manager/update_goal` | `/goal_manager/result`, `/goal_manager/goals_list` |
| **Xóa Goal** | `/goal_manager/delete_goal` | `/goal_manager/result`, `/goal_manager/goals_list` |
| **Lấy danh sách Goal** | `/goal_manager/get_goals_req` | `/goal_manager/goals_list`, `/goal_manager/result` |

---

## 3. Định Dạng Payload JSON & Hướng Dẫn Sử Dụng

### 3.1. Tạo Goal Mới (`Create Goal`)

Node `create_goal` hỗ trợ **2 cách** tạo điểm đích:

#### Cách A: Tự động lưu tọa độ hiện tại khi robot vừa di chuyển tới
Cho robot di chuyển đến điểm mong muốn, sau đó phát lệnh (không cần truyền `pose` hoặc đặt `"use_current_pose": true`):

```json
{
  "map_name": "lab20",
  "goal_id": "P1",
  "name": "Point 1 - Bàn 1",
  "use_current_pose": true,
  "description": "Lưu tọa độ robot đang đứng"
}
```

**Lệnh CLI mẫu:**
```bash
ros2 topic pub --once /goal_manager/create_goal std_msgs/msg/String '{"data": {"map_name": "lab20", "goal_id": "P1", "name": "Bàn 1", "use_current_pose": true}}'
```

#### Cách B: Truyền tọa độ thủ công (`pose`)
```json
{
  "map_name": "lab20",
  "goal_id": "P1",
  "name": "Point 1 - Bàn 1",
  "pose": {
    "x": 1.5,
    "y": 2.3,
    "theta": 0.0
  }
}
```

**Lệnh CLI mẫu:**
```bash
ros2 topic pub --once /goal_manager/create_goal std_msgs/msg/String '{"data": {"map_name": "lab20", "goal_id": "P1", "name": "Bàn 1", "pose": {"x": 1.5, "y": 2.3, "theta": 0.0}}}'
```

---

### 3.2. Cập Nhật Goal (`Update Goal`)
Publish vào topic `/goal_manager/update_goal`:

```json
{
  "map_name": "lab20",
  "goal_id": "P1",
  "name": "Bàn 1 (Đã đổi tên)",
  "pose": {
    "x": 1.8,
    "y": 2.5,
    "theta": 1.57
  }
}
```

**Lệnh test qua ROS 2 CLI:**
```bash
ros2 topic pub --once /goal_manager/update_goal std_msgs/msg/String '{"data": {"map_name": "lab20", "goal_id": "P1", "name": "Bàn 1 Mới", "pose": {"x": 1.8, "y": 2.5, "theta": 1.57}}}'
```

---

### 3.3. Xóa Goal (`Delete Goal`)
Publish vào topic `/goal_manager/delete_goal`:

```json
{
  "map_name": "lab20",
  "goal_id": "P1"
}
```

**Lệnh test qua ROS 2 CLI:**
```bash
ros2 topic pub --once /goal_manager/delete_goal std_msgs/msg/String '{"data": {"map_name": "lab20", "goal_id": "P1"}}'
```

---

### 3.4. Lấy Danh Sách Goal (`Get/List Goals`)
Publish vào topic `/goal_manager/get_goals_req`:

```json
{
  "map_name": "lab20"
}
```
*(Hoặc gửi chuỗi plain string `"lab20"`)*

**Lệnh test qua ROS 2 CLI:**
```bash
ros2 topic pub --once /goal_manager/get_goals_req std_msgs/msg/String '{"data": "lab20"}'
```

**Kết quả trả về qua topic `/goal_manager/goals_list`:**
```json
{
  "status": "ok",
  "map_name": "lab20",
  "goals": [
    {
      "goal_id": "P1",
      "name": "Bàn 1",
      "pose": {
        "x": 1.5,
        "y": 2.3,
        "theta": 0.0
      },
      "description": ""
    }
  ]
}
```

---

## 4. Cách Khởi Chạy Node

### 4.1. Khởi chạy độc lập từng Node qua CLI:
```bash
# Chạy node Tạo Goal
ros2 run mybot_goal create_goal

# Chạy node Cập nhật Goal
ros2 run mybot_goal update_goal

# Chạy node Xóa Goal
ros2 run mybot_goal delete_goal

# Chạy node Lấy danh sách Goal
ros2 run mybot_goal get_goals
```

### 4.2. Khởi chạy trong file Launch Python:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='mybot_goal', executable='create_goal', name='create_goal_node', output='screen'),
        Node(package='mybot_goal', executable='update_goal', name='update_goal_node', output='screen'),
        Node(package='mybot_goal', executable='delete_goal', name='delete_goal_node', output='screen'),
        Node(package='mybot_goal', executable='get_goals', name='get_goals_node', output='screen'),
    ])
```

---

## 5. Lưu Trữ Dữ Liệu

Dữ liệu Goal của mỗi bản đồ sẽ tự động được lưu trữ bền vững tại thư mục map hệ thống (`src/mybot_navigation2/map/`) dưới tên file:

`{map_name}_goals.json`  *(Ví dụ: `lab20_goals.json`)*
