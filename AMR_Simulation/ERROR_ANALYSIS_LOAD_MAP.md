# Báo cáo Phân tích & Khắc phục Lỗi Load Map & Goal Synchronization (RCA Report)

## 📌 1. Tổng quan sự cố (Executive Summary)

* **Mô tả hiện tượng:** Khi người dùng thao tác chuyển đổi bản đồ trên màn hình giao diện UI (Tablet / Android App), danh sách các điểm mốc (**Points / Goals**) thay đổi theo bản đồ được chọn, nhưng **hình ảnh mặt bằng bản đồ (.yaml / .pgm)** thì đứng yên không thay đổi.
* **Mức độ ảnh hưởng:** Người dùng không nhìn thấy được hình ảnh bản đồ tương ứng với các điểm mốc mới, gây mất đồng bộ dữ liệu giữa UI và robot.
* **Trạng thái xử lý:** **Đã khắc phục hoàn toàn (Resolved)**.

---

## 🔍 2. Phân tích nguyên nhân gốc rễ (Root Cause Analysis - RCA)

Sự cố xảy ra do sự kết hợp của **3 nguyên nhân kỹ thuật**:

### 🔴 Nguyên nhân 1 (Chính): Node `load_map` bị Crash âm thầm ngay khi vừa khởi động
* **Mô tả kỹ thuật:** Trong tệp `load_map.py` (thuộc package `mybot_mapping`), phương thức `__init__` chứa câu lệnh khai báo tham số:
  ```python
  self.declare_parameter('use_sim_time', True)
  ```
* **Chi tiết lỗi:** Khi khởi chạy bằng ROS 2 Launch File (như `tablet_bridge.launch.py` hoặc `navigation2.launch.py`), lớp `rclpy.Node` đã tự động khai báo tham số `use_sim_time` trước đó từ launch argument. Việc tiếp tục gọi `self.declare_parameter('use_sim_time', True)` gây trùng lặp và làm ném ra ngoại lệ:
  ```text
  rclpy.exceptions.ParameterAlreadyDeclaredException: ('Parameter(s) already declared', ['use_sim_time'])
  ```
* **Hậu quả:** Node `load_map` (`map_load_proxy`) lập tức bị dừng (**exited with failure code 1**) ngay tại thời điểm bật launch file. Do node không thể chạy, topic `/map_manager/load_map` bị mất Subscriber (dẫn tới thông báo `Waiting for at least 1 matching subscription(s)...` khi publish bằng tay) và Nav2 map server không được kích hoạt nạp bản đồ mới.

---

### 🔴 Nguyên nhân 2: Lập trình viên UI bỏ qua việc phát lệnh nạp bản đồ
* **Mô tả kỹ thuật:** Phân tích log ghi nhận từ `rosbridge_websocket` cho thấy App UI khi người dùng bấm đổi map **chỉ phát 2 thông điệp**:
  1. `MAP_MANAGER_GET_AVAILABLE_MAPS_REQ_TOPIC` (`/map_manager/get_available_maps_req`)
  2. `GOAL_GET_LIST_REQ_TOPIC` (`/goal_manager/get_goals_req`)
* **Hậu quả:** UI không hề gửi thông điệp tới topic `/map_manager/load_map`. Do đó ROS 2 chỉ nhận lệnh xin danh sách điểm mốc mà không biết người dùng muốn nạp mặt bằng bản đồ mới.

---

### 🔴 Nguyên nhân 3: Xử lý tên tập tin chứa dấu chấm (`.`) bị xén ký tự
* **Mô tả kỹ thuật:** Với các bản đồ có tên chứa dấu chấm như `L1.building`, hàm `sanitize_map_name()` sử dụng biểu thức chính quy:
  ```python
  re.sub(r'[^a-zA-Z0-9_\-]', '', map_name)
  ```
* **Hậu quả:** Dấu chấm `.` bị xóa khỏi chuỗi, khiến tên map biến thành `L1building`. Hệ thống đi tìm file `L1building.yaml` (không tồn tại) thay vì `L1.building.yaml` và trả về lỗi không tìm thấy tập tin.

---

## 🛠️ 3. Chi tiết giải pháp & Mã nguồn sửa đổi (Fixes Implemented)

### 3.1. Sửa lỗi Parameter Crash trong [load_map.py](file:///home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_mapping/mybot_mapping/load_map.py)
Kiểm tra an toàn sự tồn tại của tham số trước khi khai báo và đọc dữ liệu:

```python
# Kiểm tra an toàn trong __init__
if not self.has_parameter('use_sim_time'):
    try:
        self.declare_parameter('use_sim_time', True)
    except Exception:
        pass

# Đọc giá trị tham số an toàn
use_sim_val = self.get_parameter('use_sim_time').value if self.has_parameter('use_sim_time') else True
```

### 3.2. Tự động đồng bộ Hình ảnh Map khi UI đọc Goal
Đăng ký lắng nghe thêm topic `/goal_manager/get_goals_req` trong `MapLoadProxy`. Ngay khi UI yêu cầu danh sách Goal của bản đồ mới, Backend sẽ **tự động nạp luôn file `.yaml`/`.pgm`** của bản đồ đó lên Nav2 (`/map`):

```python
# Lắng nghe thêm topic get_goals_req
self.sub_goal_req = self.create_subscription(
    String, GOAL_GET_LIST_REQ_TOPIC, self.on_goal_req, 10)

def on_goal_req(self, message: String):
    raw_name = extract_map_name(message.data)
    map_name = sanitize_map_name(raw_name)
    if map_name and map_name != self.current_loaded_map:
        self.get_logger().info(f"UI requested goals for '{map_name}'. Auto-loading map image...")
        self.load_map_by_name(map_name)
```

### 3.3. Hỗ trợ tên bản đồ chứa dấu chấm trong [command_utils.py](file:///home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_common/mybot_common/command_utils.py)
Cập nhật regex giữ nguyên dấu chấm `.`:

```python
def sanitize_map_name(map_name: str) -> str:
    if not map_name:
        return ''
    try:
        normalized = unicodedata.normalize('NFKD', map_name)
        normalized = normalized.encode('ascii', 'ignore').decode('ascii')
    except Exception:
        normalized = map_name
    return re.sub(r'[^a-zA-Z0-9_\-\.]', '', normalized)
```

### 3.4. Định dạng dữ liệu phẳng JSON Array cho UI
Cập nhật các node trong package `mybot_goal` ([get_goals.py](file:///home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_goal/mybot_goal/get_goals.py), [create_goal.py](file:///home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_goal/mybot_goal/create_goal.py), [update_goal.py](file:///home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_goal/mybot_goal/update_goal.py), [delete_goal.py](file:///home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/src/mybot_goal/mybot_goal/delete_goal.py)) phát dữ liệu chuẩn phẳng về topic `/goal_manager/available_goals`:

```json
[
  {
    "goal_id": "P1",
    "name": "Khu vực sạc",
    "map_name": "map_lab20",
    "x": 1.0,
    "y": 1.5,
    "theta": 0.0
  }
]
```

---

## 🧪 4. Quy trình thử nghiệm & Xác minh (Verification)

1. **Biên dịch Workspace:**
   ```bash
   colcon build --packages-select mybot_common mybot_goal mybot_mapping mybot_navigation2_tablet
   ```

2. **Khởi chạy hệ thống:**
   ```bash
   ros2 launch mybot_navigation2_tablet tablet_bridge.launch.py
   ```

3. **Kiểm tra trạng thái Node đang chạy:**
   ```bash
   ros2 node list | grep map_load_proxy
   ```
   *Kết quả:* Node `/map_load_proxy` đã xuất hiện và hoạt động liên tục.

4. **Kiểm tra thử nghiệm phát lệnh nạp bản đồ:**
   ```bash
   ros2 topic pub --once /map_manager/load_map std_msgs/msg/String "data: 'map_lab20'"
   ```
   *Kết quả:* Log in ra `Processing load_map request for "map_lab20"...` và topic `/map` được cập nhật dữ liệu OccupancyGrid mới.

---

## 💡 5. Khuyến nghị & Quy tắc phòng ngừa (Best Practices)

1. **Khai báo Parameter an toàn trong ROS 2 Python:** Tránh gọi trực tiếp `self.declare_parameter()` mà không kiểm tra bằng `self.has_parameter()`.
2. **Quản lý trạng thái Node:** Sau khi thực hiện sửa mã nguồn và build, bắt buộc phải tắt hẳn các launch file cũ (`Ctrl + C`) để ROS 2 nạp binary/script mới từ bộ nhớ `install/`.
3. **Đồng bộ hóa dữ liệu UI - ROS:** Luôn duy trì cơ chế tự động nạp mặt bằng bản đồ khi nhận được yêu cầu truy vấn thông tin điểm mốc liên quan.
