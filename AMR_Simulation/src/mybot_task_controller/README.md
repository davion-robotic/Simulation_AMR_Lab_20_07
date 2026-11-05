# mybot_task_controller

Package ROS 2 Python điều khiển hành trình Robot AMR thực thi theo **từng điểm Waypoint (Point-by-Point Sequential Navigation & Delivery)**:
- **Chạy theo từng điểm chỉ định, KHÔNG loop tuần hoàn liên tục**.
- Đến từng điểm $\rightarrow$ Dừng đếm thời gian chờ (Dwell time) để giao hàng/lấy hàng.
- Hoàn tất lượt giao $\rightarrow$ **Tự động quay về trạm (`RETURNING_HOME`)** $\rightarrow$ Về đến nơi chuyển sang **`IDLE`** và xuất báo cáo thống kê.

---

## 1. Biểu Đồ Trình Tự Tương Tác (Sequence Diagram)

```mermaid
sequenceDiagram
    autonumber
    actor User as Người dùng / Khách
    participant UI as Giao diện Web / Tablet
    participant TC as Task Controller
    participant Nav2 as Nav2 Action Server
    participant Status as Status Node (/robot_status_simple)

    Note over User, UI: 1. KHỞI ĐỘNG CHUYẾN ĐI
    User->>UI: Chọn Bàn 1, Bàn 2 -> Bấm [Giao Hàng]
    Note right of UI: UI tự gắn Trạm Sạc vào cuối:<br/>[Bàn 1, Bàn 2, Trạm Sạc]
    UI->>TC: /task_controller/command<br/>{command: "START_DELIVERY", waypoints: [...]}
    TC->>Nav2: Gửi Goal Bàn 1
    TC->>Status: Cập nhật status: "navigating", current_goal: "Bàn 1" (1/3)
    Status-->>UI: Hiển thị: Đang đi tới Bàn 1 (1/3)

    Note over TC, Nav2: 2. TỚI BÀN 1 & KHÁCH XÁC NHẬN LẤY HÀNG SỚM
    Nav2-->>TC: STATUS_SUCCEEDED (Đã tới Bàn 1)
    TC->>TC: Bắt đầu đếm ngược 30s
    TC->>Status: status: "waiting", dwell_time_remaining: 30
    Status-->>UI: Hiển thị đếm ngược 30s & Sáng nút [Đã lấy hàng]
    
    User->>UI: Bấm nút [✅ Đã Lấy Hàng] (ở giây thứ 18)
    UI->>TC: /task_controller/command<br/>{command: "CONFIRM_DELIVERY"}
    TC->>TC: Reset dwell_time = 0 -> Chuyển điểm kế tiếp!

    Note over TC, Nav2: 3. TỚI BÀN 2 & ĐỢI HẾT THỜI GIAN CHỜ
    TC->>Nav2: Gửi Goal Bàn 2
    TC->>Status: status: "navigating", current_goal: "Bàn 2" (2/3)
    Status-->>UI: Hiển thị: Đang đi tới Bàn 2 (2/3)
    Nav2-->>TC: STATUS_SUCCEEDED (Đã tới Bàn 2)
    TC->>TC: Đếm ngược 45s lùi dần về 0s (Khách không bấm nút)
    TC->>TC: Hết 45s -> Tự động chuyển điểm kế tiếp!

    Note over TC, Nav2: 4. CHẠY VỀ TRẠM SẠC & HOÀN THÀNH
    TC->>Nav2: Gửi Goal Trạm Sạc (wait_sec: 0)
    TC->>Status: status: "navigating", current_goal: "Trạm Sạc" (3/3)
    Status-->>UI: Hiển thị: Đang quay về Trạm Sạc (3/3)
    Nav2-->>TC: STATUS_SUCCEEDED (Đã về tới Trạm)
    TC->>Status: status: "idle", task_state: "IDLE"
    Status-->>UI: Hiển thị: Hoàn thành nhiệm vụ. Robot sẵn sàng.
```

---

## 2. Biểu Đồ Máy Trạng Thái (State Diagram)

```mermaid
stateDiagram-v2
    [*] --> IDLE : Khởi động hệ thống

    IDLE --> NAVIGATING : Nhận lệnh START_DELIVERY (Xuất phát điểm 1)

    state NAVIGATING {
        [*] --> Moving : Gửi Goal sang Nav2
        Moving --> NavFailure : Gặp vật cản / Kẹt đường
        NavFailure --> Moving : Tự động Retry (tối đa 3 lần)
    }

    NAVIGATING --> WAITING : Tới điểm giao hàng (wait_sec > 0)
    NAVIGATING --> IDLE : Tới điểm cuối cùng (Trạm sạc, wait_sec = 0)

    state WAITING {
        [*] --> DWELLING : Bắt đầu đếm ngược thời gian chờ
        DWELLING --> FinishedWait : Hết thời gian chờ (dwell_time = 0)
        DWELLING --> FinishedWait : Khách bấm [CONFIRM_DELIVERY]
    }

    WAITING --> NAVIGATING : Còn điểm tiếp theo (i + 1)

    %% Các sự kiện ngắt điều khiển từ UI
    NAVIGATING --> PAUSED : Nhận lệnh PAUSE (Khi đang di chuyển)
    WAITING --> PAUSED : Nhận lệnh PAUSE (Khi đang đếm ngược chờ)

    PAUSED --> NAVIGATING : Nhận lệnh RESUME (Tiếp tục di chuyển)
    PAUSED --> WAITING : Nhận lệnh RESUME (Tiếp tục đếm số giây còn lại)
    PAUSED --> NAVIGATING : Nhận lệnh CONFIRM_DELIVERY (Bỏ qua chờ và đi tiếp)

    NAVIGATING --> NAVIGATING : Nhận lệnh SKIP_WAYPOINT (Bỏ qua bàn hiện tại)
    
    NAVIGATING --> IDLE : Nhận lệnh CANCEL (Hủy chuyến khẩn cấp)
    WAITING --> IDLE : Nhận lệnh CANCEL (Hủy chuyến khẩn cấp)
    PAUSED --> IDLE : Nhận lệnh CANCEL (Hủy chuyến khẩn cấp)
```

---

## 3. Giao Thức Lệnh WebSocket Chuẩn Cho UI

### A. Giao Hàng Chuỗi Điểm (Multi-Point Delivery, UI tự đặt trạm sạc làm điểm cuối):
Xe đi lần lượt Bàn 1 $\rightarrow$ Bàn 2 (mỗi điểm dừng chờ đúng số giây) $\rightarrow$ Điểm cuối cùng là Trạm Sạc (chờ 0s) $\rightarrow$ Về đến trạm chuyển ngay sang **`IDLE`**:

```json
{
  "command": "START_DELIVERY",
  "map_name": "big_map2",
  "waypoints": [
    { "goal_id": "BAN_1", "name": "Bàn 1", "wait_sec": 30 },
    { "goal_id": "BAN_2", "name": "Bàn 2", "wait_sec": 45 },
    { "goal_id": "TRAM_SAC", "name": "Trạm Sạc", "wait_sec": 0 }
  ]
}
```

### B. Nút Bấm Thao Tác Nhanh:
* **Khách lấy hàng xong sớm (Đi tiếp ngay hoặc về Trạm)**:
  ```json
  { "command": "CONFIRM_DELIVERY" }
  ```
  *(Robot nhận diện cả: `"DA_LAY_HANG"`, `"CONFIRM"`, `"NEXT"`, `"SKIP_WAIT"`)*
* **Tạm dừng**: `{"command": "PAUSE"}` (đóng băng timer chờ hoặc phanh dừng xe)
* **Tiếp tục**: `{"command": "RESUME"}`
* **Bỏ qua bàn hiện tại**: `{"command": "SKIP_WAYPOINT"}`
* **Hủy khẩn cấp**: `{"command": "CANCEL"}`

---

## 4. Trạng Thái Phản Hồi Định Kỳ 1.0 Hz (`/task_controller/status`)

```json
{
  "state": "WAITING",
  "sub_state": "DWELLING",
  "mission_type": "DELIVERY",
  "mission_id": "delivery_1793762668",
  "current_waypoint_index": 0,
  "total_waypoints": 3,
  "current_goal_id": "BAN_1",
  "current_goal_name": "Bàn 1",
  "dwell_time_remaining": 24,
  "battery_percent": 96.5,
  "is_paused": false,
  "loop": false,
  "statistics": {
    "completed_count": 1,
    "skipped_count": 0,
    "elapsed_time_sec": 85
  },
  "message": "Giao hàng tại Bàn 1 (24s còn lại)",
  "timestamp": 1793762700
}
```

* **Vòng đời State**:
  - `NAVIGATING`: Đang chạy trên đường tới điểm giao hàng.
  - `WAITING` (hoặc `DELIVERING`): Đang dừng tại bàn đếm ngược dwell time để khách lấy đồ.
  - `IDLE`: Đã về đến trạm sạc an toàn, sẵn sàng cho đơn tiếp theo!

---

## 5. Cách Chạy & Kiểm Thử

```bash
source /opt/ros/humble/setup.bash
source ~/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/install/setup.bash
ros2 launch mybot_task_controller task_controller.launch.py use_sim_time:=true
```

Test giao hàng đến bàn `P1` (chờ 30s rồi tự về Home):
```bash
ros2 topic pub --once /task_controller/command std_msgs/msg/String '{"data": "{\"command\": \"START_DELIVERY\", \"goal_id\": \"P1\", \"wait_sec\": 30}"}'
```
