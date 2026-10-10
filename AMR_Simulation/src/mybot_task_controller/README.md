# mybot_task_controller

Package ROS 2 Python để nhận và xử lý các lệnh điều khiển tác vụ từ UI / Tablet (qua Rosbridge):
- **A. Tạm dừng (`PAUSE`)**
- **B. Tiếp tục (`RESUME`)**
- **C. Hủy nhiệm vụ (`CANCEL`)**

---

## 1. Các Topics & Actions

| Tên Topic / Action | Kiểu dữ liệu | Hướng | Mô tả |
|---|---|---|---|
| `/task_controller/command` | `std_msgs/msg/String` | Subscriber | Nhận lệnh điều khiển từ UI (`PAUSE`, `RESUME`, `CANCEL`) |
| `/task_controller/status` | `std_msgs/msg/String` | Publisher | Trả về trạng thái hiện tại (`IDLE`, `RUNNING`, `PAUSED`, `CANCELLED`, `COMPLETED`) dưới dạng JSON |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Publisher | Dừng robot tức thì khi nhận `PAUSE` hoặc `CANCEL` |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Action Client | Giao tiếp với Nav2 để tạm dừng, tiếp tục hoặc hủy goal |

---

## 2. Cách Build & Chạy

### Build package:
```bash
cd ~/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select mybot_task_controller
source install/setup.bash
```

### Chạy Node:
```bash
ros2 launch mybot_task_controller task_controller.launch.py
```
*(hoặc chạy trực tiếp `ros2 run mybot_task_controller task_controller_node`)*

---

## 3. Test gửi lệnh bằng ROS 2 CLI

- **Tạm dừng (PAUSE):**
  ```bash
  ros2 topic pub --once /task_controller/command std_msgs/msg/String "{data: 'PAUSE'}"
  ```

- **Tiếp tục (RESUME):**
  ```bash
  ros2 topic pub --once /task_controller/command std_msgs/msg/String "{data: 'RESUME'}"
  ```

- **Hủy nhiệm vụ (CANCEL):**
  ```bash
  ros2 topic pub --once /task_controller/command std_msgs/msg/String "{data: 'CANCEL'}"
  ```

- **Lắng nghe phản hồi trạng thái từ Robot:**
  ```bash
  ros2 topic echo /task_controller/status
  ```
