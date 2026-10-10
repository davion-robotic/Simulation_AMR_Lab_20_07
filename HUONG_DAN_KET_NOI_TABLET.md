# 📱 Hướng Dẫn Kết Nối Thực Tế Tablet Android Với Máy Tính/Robot (USB Tethering - RNDIS & ADB)

Tài liệu này hướng dẫn chi tiết cách thiết lập, kết nối và tự động hóa truyền dữ liệu giữa **Tablet Android (HMI)** và **Máy tính điều khiển Robot (Host PC / Single Board Computer / IPC)** qua cáp USB sử dụng giao thức **RNDIS (USB Tethering)** và **ADB Port Forwarding**.

---

## 📑 MỤC LỤC
1. [Tổng Quan & Cấu Hình Đã Kiểm Chứng](#1-tổng-quan--cấu-hình-đã-kiểm-chứng)
2. [Các Bước Thiết Lập Phần Cứng & Android](#2-các-bước-thiết-lập-phần-cứng--android)
3. [Kiểm Tra Kết Nối Trên Máy Tính Linux (Host PC)](#3-kiểm-tra-kết-nối-trên-máy-tính-linux-host-pc)
4. [Các Phương Thức Truyền Dữ Liệu Thực Tế](#4-các-phương-thức-truyền-dữ-liệu-thực-tế)
5. [Tự Động Hóa Kết Nối Trên Robot (Systemd Service / Scripts)](#5-tự-động-hóa-kết-nối-trên-robot-systemd-service--scripts)
6. [Xử Lý Sự Cố Thường Gặp (Troubleshooting)](#6-xử-lý-sự-cố-thường-gặp-troubleshooting)

---

## 1. TỔNG QUAN & CẤU HÌNH ĐÃ KIỂM CHỨNG

Phương pháp kết nối qua **USB Tethering (RNDIS)** biến cổng USB thành card mạng Ethernet ảo giữa Tablet và Máy tính. Dữ liệu được truyền qua giao thức IP nội bộ với độ trễ thấp ($\approx 1-3\text{ ms}$) và ổn định vượt trội so với kết nối Wi-Fi trong môi trường công nghiệp.

### 📌 Thông Số Kiểm Chứng Thực Tế:
- **Thiết bị Tablet**: Samsung Android Tablet (`Serial: R9TR60M137J`)
- **Card mạng USB Host PC**: `enxda89d05cae36`
- **Địa chỉ IP Host PC (Máy tính)**: `10.74.18.115`
- **Địa chỉ IP Tablet**: `10.74.18.166` (giao diện `rndis0`)
- **Độ trễ trung bình**: `1.8ms - 3.2ms` (Tỷ lệ mất gói 0%)

---

## 2. CÁC BƯỚC THIẾT LẬP PHẦN CỨNG & ANDROID

### Bước 1: Cắm Cáp USB Truyền Dữ Liệu
- Đảm bảo sử dụng **cáp USB hỗ trợ truyền dữ liệu (Data Cable)** chất lượng tốt (không dùng loại cáp chỉ sạc).

### Bước 2: Bật USB Tethering & USB Debugging
1. Trên Tablet vào **Cài đặt (Settings)** $\rightarrow$ **Mạng & Internet / Kết nối (Network & Internet / Connections)** $\rightarrow$ **Điểm phát sóng cá nhân & Chia sẻ mạng (Hotspot & Tethering)**.
2. Bật **Chia sẻ mạng qua USB (USB Tethering)**.
3. Kích hoạt **Tùy chọn nhà phát triển (Developer Options)**:
   - Vào **Thông tin điện thoại** $\rightarrow$ Chạm 7 lần vào **Số hiệu bản dựng (Build Number)**.
   - Vào **Cài đặt** $\rightarrow$ **Tùy chọn nhà phát triển** $\rightarrow$ Bật **Gỡ lỗi USB (USB Debugging)**.
   - Tìm mục **Cấu hình USB mặc định (Default USB configuration)** $\rightarrow$ Chọn **Chia sẻ mạng qua USB (USB Tethering)**.

---

## 3. KIỂM TRA KẾT NỐI TRÊN MÁY TÍNH LINUX (HOST PC)

Mở Terminal trên Ubuntu/Linux và thực hiện các lệnh sau:

### 1. Kiểm tra danh sách card mạng:
```bash
ip a
```
*Kết quả sẽ xuất hiện interface dạng `usb0`, `rndis0` hoặc `enxda...` kèm địa chỉ IP (ví dụ `10.74.18.115/24`).*

### 2. Kiểm tra ADB kết nối:
```bash
adb devices
```
*Màn hình hiển thị mã serial của thiết bị Tablet (ví dụ: `R9TR60M137J  device`).*

### 3. Kiểm tra Ping kết nối mạng:
```bash
# Ping sang địa chỉ IP của Tablet
ping -c 4 10.74.18.166
```

---

## 4. CÁC PHƯƠNG THỨC TRUYỀN DỮ LIỆU THỰC TẾ

### Phương Thức A: Truy Cập Web UI HMI (Trình Duyệt Tablet $\leftrightarrow$ Web Server Host PC)
Phù hợp nhất khi Tablet đóng vai trò làm màn hình giao diện điều khiển (HMI Web App).

1. **Trên Host PC**: Chạy Web Server (Next.js, Node.js, Python Flask/FastAPI, Nginx, v.v.) tại cổng (port) mong muốn, ví dụ `3000` hoặc `8080`.
2. **Trên Tablet**: Mở trình duyệt Web (Chrome hoặc Fully Kiosk Browser) và gõ địa chỉ:
   ```text
   http://10.74.18.115:3000
   ```

### Phương Thức B: Kết Nối ROS / ROS2 WebSocket (rosbridge)
Cho phép Web UI trên Tablet truyền/nhận Topic, Service, Action trực tiếp với hệ thống ROS2 trên Robot.

1. **Khởi chạy rosbridge_server trên Robot PC**:
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```
2. **Web UI trên Tablet kết nối WebSocket**:
   ```javascript
   const ros = new ROSLIB.Ros({
       url: 'ws://10.74.18.115:9090'
   });
   ```

### Phương Thức C: Chuyển Tiếp Port ADB Reverse (ADB Port Forwarding)
Giúp Tablet có thể truy cập Web Server trên Host PC bằng địa chỉ `localhost` quen thuộc.

```bash
# Chuyển tiếp cổng 8080 và 9090
adb -s R9TR60M137J reverse tcp:8080 tcp:8080
adb -s R9TR60M137J reverse tcp:9090 tcp:9090
```
- Khi đó trên Tablet truy cập: `http://localhost:8080` hoặc `ws://localhost:9090`.

---

## 5. TỰ ĐỘNG HÓA KẾT NỐI TRÊN ROBOT (SYSTEMD SERVICE / SCRIPTS)

Để đảm bảo mỗi khi bật Robot hoặc cắm lại cáp USB, kết nối ADB Reverse & kiểm tra mạng tự động chạy:

### Tạo Script Tự Động Kết Nối: `auto_connect_tablet.sh`
```bash
cat << 'EOF' > ~/auto_connect_tablet.sh
#!/bin/bash
echo "[Tablet Setup] Đang kiểm tra thiết bị ADB..."
adb wait-for-device

echo "[Tablet Setup] Thiết lập ADB Reverse Port Forwarding..."
adb reverse tcp:8080 tcp:8080
adb reverse tcp:9090 tcp:9090

echo "[Tablet Setup] Kiểm tra IP mạng USB Tethering..."
TABLET_IP=$(adb shell ip a show rndis0 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)
echo "[Tablet Setup] Đã kết nối Tablet thành công! IP Tablet: $TABLET_IP"
EOF

chmod +x ~/auto_connect_tablet.sh
```

Chạy script khi cần:
```bash
~/auto_connect_tablet.sh
```

---

## 6. XỬ LÝ SỰ CỐ THƯỜNG GẶP (TROUBLESHOOTING)

| Hiện Tượng | Nguyên Nhân | Cách Xử Lý |
| :--- | :--- | :--- |
| **Không thấy card mạng `usb0`/`enx...`** | Cáp USB chỉ có tính năng sạc hoặc chưa bật USB Tethering. | Đổi cáp USB chuẩn data. Bật "USB Tethering" trong Cài đặt Tablet. |
| **Tablet hiển thị `unauthorized` khi gõ `adb devices`** | Chưa xác nhận ủy quyền USB Debugging trên màn hình Tablet. | Nhìn vào màn hình Tablet, tích chọn *"Always allow from this computer"* và bấm **OK**. |
| **Ping thông nhưng không vào được Web/Port** | Tường lửa (`ufw`) trên Ubuntu đang chặn kết nối. | Chạy lệnh mở cổng: `sudo ufw allow 8080/tcp` và `sudo ufw allow 9090/tcp`. |
| **Mất kết nối khi rút cáp USB cắm lại** | Android tự trả về chế độ "Just Charging". | Đặt "Default USB configuration" thành "USB Tethering" trong Tùy chọn nhà phát triển. |
| **Tablet bị cạn pin khi cắm vào PC dài ngày** | Cổng USB PC không đủ dòng sạc. | Dùng **Powered USB Hub** (Hub USB có cấp nguồn ngoài) hoặc **Cáp Y-Splitter OTG PD**. |

---
*Tài liệu được khởi tạo và kiểm chứng trực tiếp trên hệ thống AMR Simulation.*
