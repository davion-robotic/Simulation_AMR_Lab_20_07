# mybot_mode_manager

Package ROS 2 Python quản lý và chuyển đổi các chế độ hoạt động cốt lõi của Robot AMR:
- **`MAPPING`**: Tự động kích hoạt SLAM Mapping (`mapping.launch.py`) để quét tạo bản đồ.
- **`LOCALIZATION`**: Tự động kích hoạt Standalone AMCL (`localization_launch.py`) để định vị phục vụ dạy điểm, vẽ tường ảo, xóa noise.
- **`NAVIGATION`**: Tự động kích hoạt Nav2 Navigation (`navigation2.launch.py`) để điều hướng tự động.
- **`IDLE`**: Dừng toàn bộ các tiến trình mapping/navigation và dập phanh an toàn.

---

## 1. Vai Trò Cốt Lõi (Central Coordinator)

Trong hệ thống AMR, các hệ thống con lớn (**SLAM**, **AMCL Localization**, **Nav2 Navigation**) không bao giờ được chạy đồng thời vì sẽ gây xung đột chiếm dụng cảm biến LiDAR và đè gãy cây tọa độ TF (`map -> odom`).

`mode_manager_node` đóng vai trò là **"Nhạc trưởng" điều phối toàn hệ thống**:
- **Nhận lệnh chuyển chế độ** từ Tablet/Web UI thông qua WebSocket.
- **Quản lý vòng đời** (bật/tắt an toàn) các launch file lớn tương ứng mà không cần SSH vào robot.
- **Phanh dừng xe khẩn cấp** trước khi chuyển đổi trạng thái để đảm bảo an toàn cơ khí.
- **Báo cáo phản hồi liên tục** trạng thái hiện tại lên UI (tránh người dùng bấm loạn nút khi hệ thống đang khởi động).

---

## 2. Máy Trạng Thái Hữu Hạn (Finite State Machine - FSM)

```mermaid
stateDiagram-v2
    [*] --> IDLE : Khởi động Node (Mặc định)

    IDLE --> SWITCHING : Nhận lệnh MAPPING / LOCALIZATION / NAVIGATION
    MAPPING --> SWITCHING : Đổi sang Mode khác
    LOCALIZATION --> SWITCHING : Đổi sang Mode khác
    NAVIGATION --> SWITCHING : Đổi sang Mode khác

    state SWITCHING {
        [*] --> LockUI : Khóa nút bấm UI (State: SWITCHING)
        LockUI --> StopMotion : Phanh dừng xe khẩn cấp (cmd_vel = 0)
        StopMotion --> KillOldPG : Diệt sạch Process Group của tiến trình cũ
        KillOldPG --> SpawnNew : Khởi chạy Launch file mới (os.setsid)
        SpawnNew --> HealthCheck : Kiểm tra tiến trình sống (poll() is None)
    }

    SWITCHING --> MAPPING : Khởi chạy SLAM thành công
    SWITCHING --> LOCALIZATION : Khởi chạy Standalone AMCL thành công
    SWITCHING --> NAVIGATION : Khởi chạy Full Nav2 thành công
    SWITCHING --> IDLE : Thất bại hoặc có lệnh đưa về IDLE
```

---

## 3. Kiến Trúc 5 Tầng Xử Lý Bên Trong Code

```mermaid
flowchart TD
    subgraph Layer1 ["1. Tầng Giao Tiếp (ROS 2 Interfaces)"]
        SubMode["/system_mode/set_mode<br/>(Nhận JSON / String)"]
        PubState["/system_mode/state<br/>(Phát trạng thái 1.0 Hz)"]
        PubCmd["/cmd_vel<br/>(Phanh dừng xe)"]
    end

    subgraph Layer2 ["2. Tầng Phân Tích & Xác Thực (Parser)"]
        Parser["parse_payload()<br/>Trích xuất target_mode & map_name"]
        Validator{"Kiểm tra hợp lệ?<br/>MAPPING | LOCALIZATION | NAVIGATION | IDLE"}
    end

    subgraph Layer3 ["3. Tầng Luồng Nền Độc Lập (Async Threading & Lock)"]
        Worker["threading.Thread (_execute_mode_switch)<br/>Không làm block vòng lặp rclpy.spin()"]
        Mutex["threading.Lock()<br/>Ngăn chặn race-condition khi bấm nút dồn dập"]
    end

    subgraph Layer4 ["4. Tầng Quản Trị Tiến Trình (Process Group Management)"]
        Terminator["_terminate_current_process()<br/>os.killpg(pgid, SIGINT -> SIGKILL)"]
        Resolver["_resolve_map_path()<br/>Tìm đường dẫn file *.yaml"]
    end

    subgraph Layer5 ["5. Tầng Khởi Động Tiến Trình Con (Subprocess Runners)"]
        P_Map["_start_mapping_mode()<br/>mapping.launch.py"]
        P_Loc["_start_localization_mode()<br/>localization_launch.py"]
        P_Nav["_start_navigation_mode()<br/>navigation2.launch.py"]
    end

    SubMode --> Parser --> Validator
    Validator --> Worker
    Worker --> Mutex
    Mutex --> Terminator
    Terminator --> Resolver
    Resolver --> P_Map & P_Loc & P_Nav
    P_Map & P_Loc & P_Nav -. Cập nhật trạng thái .-> PubState
    Terminator -. Phanh xe .-> PubCmd
```

---

## 4. Bốn Cơ Chế Kỹ Thuật Quan Trọng Nhất Trong Code

### 1. Cơ Chế Diệt Tiến Trình Con Bằng Process Group (`os.killpg`)
- **Vấn đề thực tế:** Khi chạy `ros2 launch`, lệnh này sẽ sinh ra hàng chục tiến trình con (`amcl`, `map_server`, `bt_navigator`, `planner_server`...). Nếu chỉ dùng `process.terminate()`, hệ điều hành chỉ tắt được tiến trình cha, các tiến trình con vẫn sống ngầm ("zombie") và tiếp tục chiếm LiDAR/TF.
- **Giải pháp trong code (L218-L238):**
  1. Khi khởi chạy (`subprocess.Popen`), gán cờ `preexec_fn=os.setsid` để biến toàn bộ các node con vào cùng một **Process Group ID (PGID)** mới.
  2. Khi dừng, lấy `pgid = os.getpgid(self.current_process.pid)` và gửi tín hiệu:  
     `os.killpg(pgid, signal.SIGINT)` $\rightarrow$ Chờ 3s, nếu còn sống thì `os.killpg(pgid, signal.SIGKILL)`.  
  $\rightarrow$ Đảm bảo dọn dẹp sạch sẽ 100% không để sót bất kỳ tiến trình nào chạy ngầm!

### 2. Cơ Chế Chuyển Mode Bất Đồng Bộ (Non-blocking Async Switching)
- Việc bật/tắt Nav2 hoặc SLAM tốn từ 2 đến 4 giây (đợi nạp map, khởi tạo lifecycle).
- Nếu chạy trực tiếp trong callback `on_set_mode_received`, nó sẽ làm "đóng băng" toàn bộ vòng xoay của node (`rclpy.spin()`), khiến WebSocket bị timeout hoặc disconnect.
- **Giải pháp trong code (L153-L157):**  
  Node tách việc chuyển đổi sang một `threading.Thread` riêng, kết hợp khóa `threading.Lock()`. Nhờ đó, node vẫn duy trì bắn nhịp tim 1Hz lên `/system_mode/state` mượt mà trong khi tiến trình ngầm đang khởi tạo.

### 3. Cơ Chế An Toàn Chuyển Động (`stop_robot_motion`)
- Trước khi tắt tiến trình cũ ở dòng L167, node tự động bắn 3 lần gói tin vận tốc bằng `0.0` vào `/cmd_vel` để đảm bảo xe dừng hẳn trước khi chuyển trạng thái, tránh việc robot bị trôi trớn do mất controller đột ngột.

### 4. Cơ Chế Tự Động Phân Giải Bản Đồ (`_resolve_map_path`)
- Hàm này tự động phân giải tên map: người dùng gửi `"map_lab20"`, `"map_lab20.yaml"` hay `"map_lab20.pgm"` đều được chuẩn hóa.
- Quét tìm theo thứ tự: thư mục cài đặt share của package $\rightarrow$ thư mục source `src/mybot_navigation2/map/` $\rightarrow$ tự động fallback về map mặc định nếu file không tồn tại.

---

## 5. Bảng Tổng Hợp Các Chế Độ (System Modes)

| Mode | Ý Nghĩa Nghiệp Vụ | Launch File Được Kích Hoạt | Ứng Dụng Thực Tế |
| :--- | :--- | :--- | :--- |
| **`IDLE`** | Trạng thái nghỉ | Không có (Dọn dẹp sạch mọi tiến trình) | Xe đứng rảnh rỗi tại chỗ, bảo tồn CPU và pin. |
| **`MAPPING`** | Quét bản đồ mới | `mybot_navigation2/launch/mapping.launch.py` | Lái xe vòng quanh xưởng để quét và lưu map. |
| **`LOCALIZATION`** | Định vị độc lập | `nav2_bringup/launch/localization_launch.py` | **Vẽ tường ảo, xóa noise, dạy vị trí các trạm/bàn**. |
| **`NAVIGATION`** | Điều hướng tự động | `mybot_navigation2/launch/navigation2.launch.py` | Chạy nhiệm vụ giao hàng, tuần tra tự động. |
| **`SWITCHING`** | Đang chuyển giao | (Trạng thái chuyển tiếp nội bộ) | Báo hiệu cho UI khóa toàn bộ nút bấm & hiển thị spinner. |

---

## 6. Giao Thức WebSocket / ROSBridge Chuẩn Cho Tablet UI

### A. Topic Gửi Lệnh Chuyển Chế Độ (Tablet UI $\rightarrow$ Robot)
- **Topic**: `/system_mode/set_mode`
- **Kiểu dữ liệu**: `std_msgs/msg/String`
- **Gói tin Raw WebSocket JSON**:

#### 1. Bật Mapping:
```json
{
  "op": "publish",
  "topic": "/system_mode/set_mode",
  "type": "std_msgs/msg/String",
  "msg": { "data": "MAPPING" }
}
```

#### 2. Bật Standalone Localization (Setup, Dạy Điểm & Vẽ Tường Ảo):
```json
{
  "op": "publish",
  "topic": "/system_mode/set_mode",
  "type": "std_msgs/msg/String",
  "msg": { "data": "{\"mode\": \"LOCALIZATION\", \"map_name\": \"map_lab20\"}" }
}
```

#### 3. Bật Navigation:
```json
{
  "op": "publish",
  "topic": "/system_mode/set_mode",
  "type": "std_msgs/msg/String",
  "msg": { "data": "{\"mode\": \"NAVIGATION\", \"map_name\": \"map_lab20\"}" }
}
```

#### 4. Dừng / IDLE:
```json
{
  "op": "publish",
  "topic": "/system_mode/set_mode",
  "type": "std_msgs/msg/String",
  "msg": { "data": "IDLE" }
}
```

---

### B. Topic Phản Hồi Trạng Thái (Robot $\rightarrow$ Tablet UI)
- **Topic**: `/system_mode/state`
- **Kiểu dữ liệu**: `std_msgs/msg/String`
- **Tần số**: Bắn ngay khi chuyển trạng thái + lặp lại định kỳ 1.0 Hz (để Tablet kết nối lại vẫn nhận được).
- **Subscribe từ Tablet UI**:
```json
{
  "op": "subscribe",
  "topic": "/system_mode/state",
  "type": "std_msgs/msg/String"
}
```
- **Các giá trị trạng thái**:
  - `"SWITCHING"`: Đang trong quá trình tắt tiến trình cũ và khởi chạy tiến trình mới (UI hiển thị loading/spinner và khóa nút).
  - `"MAPPING"`: Chế độ SLAM đã sẵn sàng (UI mở khóa, hiển thị trạng thái Mapping).
  - `"LOCALIZATION"`: Chế độ Định vị độc lập đã sẵn sàng (UI mở công cụ dạy điểm, sửa map).
  - `"NAVIGATION"`: Chế độ Nav2 đã sẵn sàng (UI mở khóa, hiển thị trạng thái Navigation).
  - `"IDLE"`: Hệ thống đang nghỉ (UI hiển thị IDLE).

---

## 7. Quy Trình UI Frontend Khuyến Nghị

```typescript
// Định nghĩa kiểu dữ liệu TypeScript
export type SystemMode = "MAPPING" | "LOCALIZATION" | "NAVIGATION" | "IDLE";
export type SystemState = SystemMode | "SWITCHING";

// Luồng xử lý nút bấm:
function setRobotMode(targetMode: SystemMode, mapName?: string) {
  // 1. Khoá nút, hiện loading
  setLoading(true);
  
  // 2. Publish gói tin lên /system_mode/set_mode
  setModeTopic.publish({
    data: JSON.stringify({ mode: targetMode, map_name: mapName || "map_lab20" })
  });
}

// Lắng nghe topic /system_mode/state
stateTopic.subscribe((message) => {
  const currentState = message.data as SystemState;
  
  if (currentState === "SWITCHING") {
    // Đang chuyển đổi -> tiếp tục hiện loading
    setLoading(true);
  } else {
    // Đã chuyển xong (MAPPING | LOCALIZATION | NAVIGATION | IDLE) -> tắt loading
    setLoading(false);
    setCurrentMode(currentState);
  }
});
```

---

## 8. Cách Build & Chạy

### Build package:
```bash
cd ~/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select mybot_mode_manager --symlink-install
source install/setup.bash
```

### Chạy Node:
```bash
ros2 launch mybot_mode_manager mode_manager.launch.py use_sim_time:=true
```

---

## 9. Test Bằng ROS 2 CLI

```bash
# 1. Mở một terminal theo dõi trạng thái:
ros2 topic echo /system_mode/state

# 2. Bật Mapping:
ros2 topic pub --once /system_mode/set_mode std_msgs/msg/String '{"data": "MAPPING"}'

# 3. Bật Localization (Setup mode):
ros2 topic pub --once /system_mode/set_mode std_msgs/msg/String '{"data": "{\"mode\": \"LOCALIZATION\", \"map_name\": \"map_lab20\"}"}'

# 4. Bật Navigation:
ros2 topic pub --once /system_mode/set_mode std_msgs/msg/String '{"data": "{\"mode\": \"NAVIGATION\", \"map_name\": \"map_lab20\"}"}'

# 5. Về IDLE:
ros2 topic pub --once /system_mode/set_mode std_msgs/msg/String '{"data": "IDLE"}'
```
