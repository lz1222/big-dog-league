cmake_minimum_required(VERSION 3.8)
project(rk_arm)

# D1 驱动：接口与安全核心可离线测试；运行节点才链接 Unitree DDS。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(CTest)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rk_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(rk_arm_core
  src/arm_safety_core.cpp
  src/d1_observed_command.cpp
  src/d1_feedback_parser.cpp
  src/d1_command_encoder.cpp
  src/single_writer_guard.cpp)
target_include_directories(rk_arm_core PUBLIC include)
target_link_libraries(rk_arm_core PUBLIC Threads::Threads)

set(UNITREE_D1_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_d1_sdk" CACHE PATH "Unitree D1 SDK source root")
set(UNITREE_SDK2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2" CACHE PATH "Unitree SDK2 source root")
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(D1_SDK_ARCH aarch64)
else()
  set(D1_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(UNITREE_SDK2_LIBRARY "${UNITREE_SDK2_ROOT}/lib/${D1_SDK_ARCH}/libunitree_sdk2.a")
set(UNITREE_DDS_LIBRARY_DIR "${UNITREE_SDK2_ROOT}/thirdparty/lib/${D1_SDK_ARCH}")
if(NOT EXISTS "${UNITREE_D1_SDK_ROOT}/src/msg/ArmString_.hpp" OR NOT EXISTS "${UNITREE_SDK2_LIBRARY}")
  message(FATAL_ERROR "D1 SDK or architecture-matched SDK2 library is missing")
endif()

add_executable(d1_dds_driver_node
  src/d1_dds_driver_node.cpp
  "${UNITREE_D1_SDK_ROOT}/src/msg/ArmString_.cpp"
  "${UNITREE_D1_SDK_ROOT}/src/msg/PubServoInfo_.cpp")
target_include_directories(d1_dds_driver_node PRIVATE include "${UNITREE_D1_SDK_ROOT}/src" "${UNITREE_SDK2_ROOT}/include" "${UNITREE_SDK2_ROOT}/thirdparty/include" "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
target_link_libraries(d1_dds_driver_node rk_arm_core "${UNITREE_SDK2_LIBRARY}" "${UNITREE_DDS_LIBRARY_DIR}/libddsc.so" "${UNITREE_DDS_LIBRARY_DIR}/libddscxx.so" Threads::Threads rt)
target_link_options(d1_dds_driver_node PRIVATE "-Wl,--disable-new-dtags")
set_target_properties(d1_dds_driver_node PROPERTIES BUILD_RPATH "${UNITREE_DDS_LIBRARY_DIR}" INSTALL_RPATH "${UNITREE_DDS_LIBRARY_DIR}")

add_executable(arm_manual_control_node src/arm_manual_control_node.cpp)
target_link_libraries(arm_manual_control_node rk_arm_core)
ament_target_dependencies(arm_manual_control_node rclcpp rk_interfaces std_msgs)

install(TARGETS rk_arm_core d1_dds_driver_node arm_manual_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch tools DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING)
  foreach(test_name test_arm_safety_core test_feedback_parser test_command_encoder test_observed_command test_single_writer_guard)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE rk_arm_core)
    if(test_name STREQUAL "test_observed_command")
      target_compile_definitions(${test_name} PRIVATE RK_ARM_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    endif()
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

ament_package()
