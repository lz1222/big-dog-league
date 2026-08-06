cmake_minimum_required(VERSION 3.5)
project(rk_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineTrack.msg"
  "msg/SpecialTargetDetection.msg"
  "msg/SignDetection.msg"
  "msg/SignDetectionArray.msg"
  "msg/ItemTag.msg"
  "msg/ItemTagArray.msg"
  "msg/ColorObjectDetection.msg"
  "action/ExecuteMotion.action"
  "action/ExecuteArmTask.action"
  "action/RunMission.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
