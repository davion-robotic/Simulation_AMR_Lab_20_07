# Hệ Thống Mô Phỏng & Điều Khiển Robot Tự Hành AMR (AMR Simulation & Control System)

Workspace ROS 2 toàn diện phục vụ mô phỏng, định vị (Localization), tạo bản đồ (SLAM), điều hướng tự hành (Nav2 Navigation), quản lý điểm đến (Waypoint/Goal Management), tích hợp giao diện Web/Tablet qua ROSBridge và điều phối đội xe (Open-RMF).

---

## 📑 Mục Lục
1. [Cấu Trúc Workspace & Danh Mục Package](#-cấu-trúc-workspace--danh-mục-package)
2. [Yêu Cầu Hệ Thống & Cài Đặt Phụ Thuộc](#-yêu-cầu-hệ-thống--cài-đặt-phụ-thuộc)
3. [Hướng Dẫn Build Dự Án](#-hướng-dẫn-build-dự-án)
4. [Hướng Dẫn Vận Hành Từng Bước (Quick Start)](#-hướng-dẫn-vận-hành-từng-bước-quick-start)
   - [Kịch bản 1: Khởi động Mô phỏng Gazebo & RViz](#bước-1-khởi-động-mô-phỏng-gazebo--rviz)
   - [Kịch bản 2: Tạo Bản Đồ (SLAM Mapping & Lưu Map)](#kịch-bản-2-tạo-bản-đồ-slam-mapping--lưu-map)
   - [Kịch bản 3: Khởi động Backend Tablet/Web Bridge](#kịch-bản-3-khởi-động-cầu-nối-web--tablet-bridge)
   - [Kịch bản 4: Định Vị Robot (AMCL & Initial Pose)](#kịch-bản-4-định-vị-robot-amcl--initial-pose)
   - [Kịch bản 5: Thiết Lập & Quản Lý Điểm Đích (Goal Management)](#kịch-bản-5-thiết-lập--quản-lý-điểm-đích-goal-management)
   - [Kịch bản 6: Điều Hướng Tự Động Với Nav2](#kịch-bản-6-điều-hướng-tự-động-với-nav2)
   - [Kịch bản 7: Quản Lý Đội Xe Open-RMF](#kịch-bản-7-quản-lý-đội-xe-open-rmf)
5. [Bảng Tra Cứu Topic & Payload JSON](#-bảng-tra-cứu-topic--payload-json)
6. [Script Dọn Dẹp Tiến Trình (Reset Tool)](#-script-dọn-dẹp-tiến-trình-reset-tool)

---

## 📦 Cấu Trúc Workspace & Danh Mục Package

Thư mục mã nguồn `src/` gồm 11 packages được module hóa theo từng chức năng:

```text
src/
├── mybot_navigation2/         # Mô tả robot (URDF/Xacro), World Gazebo, cấu hình Nav2/EKF và launch chính
├── mybot_localization/        # Vòng đời định vị AMCL, đặt Initial Pose, Reset Pose
├── mybot_mapping/             # Vòng đời SLAM, load map, lưu bản đồ và lưu map chỉnh sửa
├── mybot_goal/                # CRUD điểm đến (Waypoint/Goal) lưu dưới dạng JSON
├── mybot_rmf/                 # Tích hợp Open-RMF Fleet Adapter và Navigation Graph
├── mybot_navigation2_tablet/  # Cầu nối backend toàn diện (WebSocket 9090 + các node quản lý)
├── mybot_common/              # Constants, danh sách topic chuẩn hóa, tiện ích xử lý JSON/Command
├── mybot_status/              # Publish trạng thái hoạt động robot
├── mybot_navigation2_py/      # Tiện ích Python hỗ trợ Camera, Cartographer
├── ros2_laser_scan_matcher/   # C++ node tính toán Laser Odometry từ cảm biến LiDAR 2D
└── csm/                       # C/C++ library: Canonical Scan Matcher (ICP)
```

---

## ⚙️ Yêu Cầu Hệ Thống & Cài Đặt Phụ Thuộc

* **Hệ điều hành**: Ubuntu 22.04 LTS (hoặc Ubuntu 20.04)
* **ROS 2**: ROS 2 Humble Hawksbill (hoặc Iron / Rolling)
* **Mô phỏng**: Gazebo Classic 11
* **Nav2 & RMF**: `nav2-bringup`, `slam-toolbox`, `rosbridge-server`, `rmf-fleet-adapter`

### Cài đặt công cụ và thư viện cần thiết:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-rosbridge-server \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rmf-fleet-adapter
```

---

## 🔨 Hướng Dẫn Build Dự Án

```bash
# 1. Chuyển vào thư mục workspace
cd ~/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation

# 2. Cập nhật rosdep (nếu cần)
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 3. Build toàn bộ workspace
colcon build --symlink-install

# 4. Nạp biến môi trường
source install/setup.bash
```

---

## 🚀 Hướng Dẫn Vận Hành Từng Bước (Quick Start)

### Bước 1: Khởi động Mô phỏng Gazebo & RViz
Mở **Terminal 1**:
```bash
source /opt/ros/humble/setup.bash
source ~/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/install/setup.bash
ros2 launch mybot_navigation2 launch_sim.launch.py
```
> Cửa sổ mô phỏng Gazebo 3D và RViz2 hiển thị mô hình robot AMR được khởi chạy.

---

### Kịch bản 2: Tạo Bản Đồ (SLAM Mapping & Lưu Map)

1. **Khởi chạy SLAM Toolbox (Terminal 2):**
   ```bash
   source install/setup.bash
   ros2 launch mybot_navigation2 mapping.launch.py use_sim_time:=true
   ```

2. **Lái robot quét không gian (Terminal 3):**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **Lưu bản đồ khi hoàn tất (Terminal 4):**
   ```bash
   ros2 topic pub --once /web/save_map_cmd std_msgs/msg/String '{"data": "{\"map_name\": \"my_new_map\"}"}'
   ```
   > File map sẽ được lưu tại `src/mybot_navigation2/map/my_new_map.yaml` và `my_new_map.pgm`.

---

### Kịch bản 3: Khởi Động Cầu Nối Web / Tablet Bridge
Mở **Terminal 2**:
```bash
source install/setup.bash
ros2 launch mybot_navigation2_tablet tablet_bridge.launch.py
```
> Node này tự động kích hoạt **ROSBridge WebSocket (Port 9090)**, **Robot Status Publisher**, **Map Manager**, **Localization Lifecycle Manager**, **Goal Manager** và **Topology Manager**.

---

### Kịch bản 4: Định Vị Robot (AMCL & Initial Pose)

1. **Bật AMCL độc lập cho bản đồ `map_lab20`:**
   ```bash
   ros2 topic pub --once /localization/start std_msgs/msg/String '{"data": "{\"map_name\": \"map_lab20\", \"use_sim_time\": true}"}'
   ```

2. **Nạp Initial Pose theo vị trí hiện tại của robot:**
   ```bash
   ros2 topic pub --once /localization/initial_pose std_msgs/msg/String '{"data": "{\"use_current_pose\": true}"}'
   ```

3. **Reset Pose về gốc tọa độ (0,0,0):**
   ```bash
   ros2 topic pub --once /localization/reset_pose std_msgs/msg/String '{"data": "{\"command\": \"reset\"}"}'
   ```

---

### Kịch bản 5: Thiết Lập & Quản Lý Điểm Đích (Goal Management)

1. **Tạo điểm Goal tự động bắt tọa độ hiện tại của robot (sau khi lái xe tới đích):**
   ```bash
   ros2 topic pub --once /goal_manager/create_goal std_msgs/msg/String '{"data": "{\"map_name\": \"map_lab20\", \"goal_id\": \"P1\", \"name\": \"Bàn 1\", \"use_current_pose\": true}"}'
   ```

2. **Tạo điểm Goal truyền tọa độ thủ công:**
   ```bash
   ros2 topic pub --once /goal_manager/create_goal std_msgs/msg/String '{"data": "{\"map_name\": \"map_lab20\", \"goal_id\": \"P2\", \"name\": \"Trạm Sạc\", \"pose\": {\"x\": 1.5, \"y\": 2.0, \"theta\": 0.0}}"}'
   ```

3. **Xem danh sách Goal của một bản đồ:**
   ```bash
   ros2 topic pub --once /goal_manager/get_goals_req std_msgs/msg/String '{"data": "map_lab20"}'
   # Lắng nghe danh sách trả về:
   ros2 topic echo --no-str-trunc /goal_manager/goals_list
   ```

4. **Xóa điểm Goal:**
   ```bash
   ros2 topic pub --once /goal_manager/delete_goal std_msgs/msg/String '{"data": "{\"map_name\": \"map_lab20\", \"goal_id\": \"P1\"}"}'
   ```

---

### Kịch bản 6: Điều Hướng Tự Động Với Nav2

1. **Khởi chạy Nav2 Navigation Stack:**
   ```bash
   source install/setup.bash
   ros2 launch mybot_navigation2 navigation2.launch.py map:=/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_navigation2/map/map_lab20.yaml use_sim_time:=true
   ```

2. **Gửi lệnh di chuyển trực tiếp đến tọa độ mục tiêu:**
   ```bash
   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
   ```

---

### Kịch bản 7: Quản Lý Đội Xe Open-RMF

1. **Chạy toàn bộ hệ thống RMF Traffic & Fleet Adapter:**
   ```bash
   source install/setup.bash
   ros2 launch mybot_rmf rmf_full.launch.py
   ```

2. **Lệnh cho robot đi tới một Waypoint trong Navigation Graph:**
   ```bash
   ros2 run mybot_rmf goto_waypoint --ros-args -p target:=table_1
   ```

---

## 📊 Bảng Tra Cứu Topic & Payload JSON

| Nghiệp Vụ | Topic Gửi Lệnh | Kiểu Dữ Liệu | Ví Dụ Payload JSON | Topic Phản Hồi / Kết Quả |
| :--- | :--- | :--- | :--- | :--- |
| **Lưu Map** | `/web/save_map_cmd` | `std_msgs/String` | `{"map_name": "lab20"}` | `/web/save_map_result` |
| **Tải Map** | `/web/load_map_cmd` | `std_msgs/String` | `{"map_name": "lab20"}` | `/web/load_map_result` |
| **Bật Định Vị** | `/localization/start` | `std_msgs/String` | `{"map_name": "lab20", "use_sim_time": true}` | `/localization/status` |
| **Tắt Định Vị** | `/localization/stop` | `std_msgs/String` | `{"command": "stop"}` | `/localization/status` |
| **Initial Pose** | `/localization/initial_pose`| `std_msgs/String` | `{"use_current_pose": true}` | `/localization/result` |
| **Reset Pose** | `/localization/reset_pose` | `std_msgs/String` | `{"command": "reset"}` | `/localization/result` |
| **Tạo Goal** | `/goal_manager/create_goal` | `std_msgs/String` | `{"map_name": "lab20", "goal_id": "P1", "name": "Bàn 1", "use_current_pose": true}` | `/goal_manager/result`, `/goal_manager/goals_list` |
| **Cập Nhật Goal**| `/goal_manager/update_goal` | `std_msgs/String` | `{"map_name": "lab20", "goal_id": "P1", "name": "Bàn 1 Mới", "pose": {"x": 1.0, "y": 2.0, "theta": 0.0}}` | `/goal_manager/result`, `/goal_manager/goals_list` |
| **Xóa Goal** | `/goal_manager/delete_goal` | `std_msgs/String` | `{"map_name": "lab20", "goal_id": "P1"}` | `/goal_manager/result`, `/goal_manager/goals_list` |
| **Lấy Danh Sách Goal** | `/goal_manager/get_goals_req` | `std_msgs/String` | `{"map_name": "lab20"}` hoặc `"lab20"` | `/goal_manager/goals_list` |
| **Trạng Thái Robot** | `/robot_status` | `std_msgs/String` | *(Robot tự publish định kỳ: pin, pose, trạng thái điều hướng)* | `/robot_status` |

---

## 🧹 Script Dọn Dẹp Tiến Trình (Reset Tool)

Khi muốn tắt toàn bộ Gazebo, RViz, ROSBridge và các node bị treo ở background để chạy lại sạch sẽ:

```bash
cd ~/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation
bash reset.sh
```

---

## 💡 Mẹo & Lưu Ý Quan Trọng

1. **`use_sim_time`**: Luôn đảm bảo tham số `use_sim_time:=true` khi chạy trên môi trường mô phỏng Gazebo.
2. **FastDDS Shared Memory**: Nếu gặp hiện tượng node không nhận topic, script `reset.sh` sẽ tự động dọn dẹp `/dev/shm/fastrtps_port*`.
3. **Lưu trữ Goal**: Các điểm Goal được lưu trực tiếp thành các file JSON tương ứng trong thư mục `src/mybot_navigation2/map/` (ví dụ: `map_lab20_goals.json`).
