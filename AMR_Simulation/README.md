# AMR_Simulation

Workspace ROS 2 cho hệ thống mô phỏng và điều khiển robot di động tự hành (AMR). Dự án này bao gồm các package chính sau:

- mybot_navigation2: mô phỏng Gazebo, SLAM, điều hướng Nav2
- mybot_mapping_stream: xử lý và stream dữ liệu bản đồ/mapping
- mybot_nav2_web_control: điều khiển tốc độ robot qua websocket hoặc terminal

## Cấu trúc workspace

- src/mybot_navigation2: các launch file, world, tham số điều hướng và bản đồ
- src/mybot_mapping_stream: package Python cho xử lý dữ liệu mapping
- src/mybot_nav2_web_control: package Python và launch file cho điều khiển web

## Yêu cầu hệ thống

- ROS 2 đã được cài đặt (ví dụ: Humble, Iron)
- Python 3
- colcon
- rosdep

## Thiết lập môi trường

1. Cài ROS 2 theo hướng dẫn chính thức của distro bạn đang dùng.
2. Cài các công cụ build:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

3. Khởi tạo rosdep nếu chưa làm:

```bash
sudo rosdep init
rosdep update
```

4. Vào thư mục workspace:

```bash
cd ~/AMR_Simulation
```

5. Cài phụ thuộc cho các package nguồn:

```bash
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Thay <distro> bằng tên ROS 2 bạn đã cài, ví dụ humble.

## Build workspace

```bash
cd ~/AMR_Simulation
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
```

## Chạy mô phỏng và mapping

### Bước 1: Khởi động môi trường mô phỏng

Terminal 1:

```bash
source /opt/ros/<distro>/setup.bash
source ~/AMR_Simulation/install/setup.bash
ros2 launch mybot_navigation2 launch_sim.launch.py
```

### Bước 2: Chạy SLAM để tạo bản đồ

Terminal 2:

```bash
source /opt/ros/<distro>/setup.bash
source ~/AMR_Simulation/install/setup.bash
ros2 launch mybot_navigation2 mapping.launch.py use_sim_time:=true
```

Sau khi mapping xong, bạn có thể lưu bản đồ theo cấu hình của SLAM package đang dùng.

## Chạy điều hướng trên bản đồ đã có

### Bước 1: Khởi động Gazebo và robot

Terminal 1:

```bash
source /opt/ros/<distro>/setup.bash
source ~/AMR_Simulation/install/setup.bash
ros2 launch mybot_navigation2 launch_sim.launch.py
```

### Bước 2: Chạy Nav2 với bản đồ đã lưu

Terminal 2:

```bash
source /opt/ros/<distro>/setup.bash
source ~/AMR_Simulation/install/setup.bash
ros2 launch mybot_navigation2 navigation2.launch.py
```

Lưu ý:
- Cần chỉnh đúng đường dẫn bản đồ trong file navigation2.launch.py hoặc truyền tham số map phù hợp.
- Sau khi launch xong, mở RViz và dùng chức năng 2D Pose Estimate để xác định vị trí ban đầu của robot trước khi gửi goal điều hướng.

## Điều khiển tốc độ robot

### Dùng websocket launch file

Terminal 3:

```bash
source /opt/ros/<distro>/setup.bash
source ~/AMR_Simulation/install/setup.bash
ros2 launch mybot_nav2_web_control Nav2_speed_control.launch.py
```

### Dùng node kiểm tra trực tiếp từ terminal

Terminal 4:

```bash
source /opt/ros/<distro>/setup.bash
source ~/AMR_Simulation/install/setup.bash
ros2 run mybot_nav2_web_control nav2_speed_control
```

## Ghi chú quan trọng

- Các launch file chính nằm trong thư mục src/mybot_navigation2/launch.
- launch_sim.launch.py dùng để khởi động mô phỏng Gazebo và spawn robot.
- mapping.launch.py dùng cho quy trình SLAM và tạo bản đồ.
- navigation2.launch.py dùng cho điều hướng dựa trên bản đồ đã có.
- Khi mở RViz, hãy thêm display Map và chọn đúng topic bản đồ để theo dõi trạng thái mapping/điều hướng.

Nếu cần, bạn có thể tiếp tục mở rộng README này bằng hướng dẫn dùng cụ thể cho từng launch file hoặc các vấn đề thường gặp khi chạy trên ROS distro khác.
