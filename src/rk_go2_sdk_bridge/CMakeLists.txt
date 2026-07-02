cmake_minimum_required(VERSION 3.5)
project(rk_go2_sdk_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  /usr/local/include
  /usr/local/include/unitree
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)

link_directories(
  /usr/local/lib
)

add_executable(go2_sdk_cmd_vel_bridge
  src/go2_sdk_cmd_vel_bridge.cpp
)

ament_target_dependencies(go2_sdk_cmd_vel_bridge
  rclcpp
  geometry_msgs
)

target_link_libraries(go2_sdk_cmd_vel_bridge
  unitree_go2_sdk
  unitree_go2_idl_cpp
  unitree_ros2_idl_cpp
  ddsc
  ddscxx
  rt
  pthread
)

install(TARGETS go2_sdk_cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
