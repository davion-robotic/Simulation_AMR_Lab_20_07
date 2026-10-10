Bước 1 — Lấy tọa độ hiện tại của robot:

bash
ros2 topic echo /amcl_pose --once

Ghi lại position.x, position.y, orientation.z, orientation.w.

Bước 2 — Tính điểm goal mới cách đó ~0.5m (để test an toàn, tránh đi quá xa lần đầu). Ví dụ nếu robot đang ở đâu đó gần x: -2.16, y: -1.79 (theo lệnh bạn vừa gõ), bạn có thể giữ nguyên hướng hiện tại và chỉ dịch nhẹ tọa độ đích, hoặc dùng đúng tọa độ đó nếu đó chính là điểm đích mong muốn.
Được, đây là cú pháp ros2 topic pub chuẩn để publish một lần lên /goal_pose. Bạn chạy trực tiếp:

bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 0.7, y: 1.5, z: 0.0}, orientation: {z: -0.316, w: 0.949}}}"

Vài lưu ý để lệnh này chạy đúng:

1. Nhớ source workspace trước (nếu terminal mới):

bash
source /opt/ros/humble/setup.bash
source ~/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/install/setup.bash

2. Copy nguyên dòng, đừng gõ lại tay — để tránh lỗi dấu ngoặc kép do gõ nhầm shell escape.

3. Kiểm tra kết quả:

Terminal đang ros2 topic echo /goal_pose (nếu có mở) sẽ hiện message này.
Robot trong RViz/Gazebo bắt đầu di chuyển tới (0.7, 1.5).
Theo dõi trạng thái:
bash
ros2 topic echo /navigate_to_pose/_action/status

4. Tọa độ x: 0.7, y: 1.5 này đang dựa trên vị trí robot cũ (x: 0.174, y: 1.510) bạn cho mình lúc trước. Nếu robot đã di chuyển nơi khác từ lúc đó, tọa độ này có thể không còn hợp lý (quá xa hoặc đi vào tường). Bạn có muốn lấy lại /amcl_pose hiện tại để cập nhật tọa độ goal cho chính xác không?