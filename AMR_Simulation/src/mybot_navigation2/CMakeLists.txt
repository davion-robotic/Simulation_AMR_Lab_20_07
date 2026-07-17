cmake_minimum_required(VERSION 3.5)
project(mybot_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    map
    param
    rviz
    description
    config
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/ControlRobotOdom.py
    src/OdomPublish.py
    src/Depht.py
    src/send_patrol.py
    src/Websocket_Client.py
    src/fake_web_cmd_vel.py
    src/Nav2_speed_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()