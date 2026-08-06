cmake_minimum_required(VERSION 3.16)
project(d1_gripper_tx_probe LANGUAGES CXX)

# 该实验 writer 默认不可构建，避免被正常工作区、比赛 launch 或误执行路径带入。
option(BUILD_D1_EXPERIMENTAL_WRITER "Build EXPERIMENTAL HARDWARE WRITER" OFF)
if(NOT BUILD_D1_EXPERIMENTAL_WRITER)
  message(FATAL_ERROR "d1_gripper_tx_probe is disabled; set -DBUILD_D1_EXPERIMENTAL_WRITER=ON explicitly")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(CTest)
find_package(Threads REQUIRED)

set(UNITREE_D1_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_d1_sdk" CACHE PATH "Unitree D1 SDK source root")
set(UNITREE_SDK2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2" CACHE PATH "Unitree SDK2 source root")
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(D1_SDK_ARCH aarch64)
else()
  set(D1_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(UNITREE_SDK2_LIBRARY "${UNITREE_SDK2_ROOT}/lib/${D1_SDK_ARCH}/libunitree_sdk2.a")
set(UNITREE_DDS_LIBRARY_DIR "${UNITREE_SDK2_ROOT}/thirdparty/lib/${D1_SDK_ARCH}")
if(NOT EXISTS "${UNITREE_D1_SDK_ROOT}/src/msg/ArmString_.hpp" OR NOT EXISTS "${UNITREE_SDK2_LIBRARY}" OR
   NOT EXISTS "${UNITREE_DDS_LIBRARY_DIR}/libddsc.so" OR NOT EXISTS "${UNITREE_DDS_LIBRARY_DIR}/libddscxx.so")
  message(FATAL_ERROR "Architecture-matched Unitree D1 SDK2/CycloneDDS files are required")
endif()

# 复用已测试的离线完整七通道模型；这些源文件没有 ROS 或 DDS 依赖。
add_library(d1_gripper_tx_core STATIC
  src/gripper_tx_core.cpp
  ../../src/rk_arm/src/arm_safety_core.cpp
  ../../src/rk_arm/src/d1_feedback_parser.cpp
  ../../src/rk_arm/src/d1_observed_command.cpp
  ../../src/rk_arm/src/single_writer_guard.cpp)
target_include_directories(d1_gripper_tx_core PUBLIC include ../../src/rk_arm/include)
target_link_libraries(d1_gripper_tx_core PUBLIC Threads::Threads)

add_executable(d1_gripper_tx_probe
  src/d1_gripper_tx_probe.cpp
  "${UNITREE_D1_SDK_ROOT}/src/msg/ArmString_.cpp"
  "${UNITREE_D1_SDK_ROOT}/src/msg/PubServoInfo_.cpp")
target_include_directories(d1_gripper_tx_probe PRIVATE
  include ../../src/rk_arm/include "${UNITREE_D1_SDK_ROOT}/src" "${UNITREE_SDK2_ROOT}/include"
  "${UNITREE_SDK2_ROOT}/thirdparty/include" "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
target_link_libraries(d1_gripper_tx_probe PRIVATE d1_gripper_tx_core "${UNITREE_SDK2_LIBRARY}"
  "${UNITREE_DDS_LIBRARY_DIR}/libddsc.so" "${UNITREE_DDS_LIBRARY_DIR}/libddscxx.so" Threads::Threads rt)
set_target_properties(d1_gripper_tx_probe PROPERTIES BUILD_RPATH "${UNITREE_DDS_LIBRARY_DIR}" INSTALL_RPATH "${UNITREE_DDS_LIBRARY_DIR}")
target_link_options(d1_gripper_tx_probe PRIVATE "-Wl,--disable-new-dtags")

if(BUILD_TESTING)
  add_executable(test_gripper_tx_core test/test_gripper_tx_core.cpp)
  target_link_libraries(test_gripper_tx_core PRIVATE d1_gripper_tx_core)
  add_test(NAME gripper_tx_core COMMAND test_gripper_tx_core)
endif()
