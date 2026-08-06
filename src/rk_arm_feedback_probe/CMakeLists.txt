cmake_minimum_required(VERSION 3.16)
project(rk_arm_feedback_probe LANGUAGES CXX)

# 纯 DDS 反馈探针：刻意不引入 ROS 2，避免 Foxy RMW 与 Unitree DDS 混载。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
find_package(Threads REQUIRED)

set(UNITREE_D1_SDK_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_d1_sdk"
    CACHE PATH "Unitree D1 SDK source root")
set(UNITREE_SDK2_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2"
    CACHE PATH "Unitree SDK2 source root")

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(D1_SDK_ARCH aarch64)
else()
  set(D1_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(UNITREE_SDK2_LIBRARY
    "${UNITREE_SDK2_ROOT}/lib/${D1_SDK_ARCH}/libunitree_sdk2.a")
set(UNITREE_DDS_LIBRARY_DIR
    "${UNITREE_SDK2_ROOT}/thirdparty/lib/${D1_SDK_ARCH}")

if(NOT EXISTS "${UNITREE_D1_SDK_ROOT}/src/msg/ArmString_.hpp")
  message(FATAL_ERROR "D1 message headers not found: ${UNITREE_D1_SDK_ROOT}")
endif()
if(NOT EXISTS "${UNITREE_SDK2_LIBRARY}")
  message(FATAL_ERROR "Architecture-matched Unitree SDK2 library not found: ${UNITREE_SDK2_LIBRARY}")
endif()
if(NOT EXISTS "${UNITREE_DDS_LIBRARY_DIR}/libddsc.so" OR
   NOT EXISTS "${UNITREE_DDS_LIBRARY_DIR}/libddscxx.so")
  message(FATAL_ERROR "Architecture-matched Unitree DDS libraries not found: ${UNITREE_DDS_LIBRARY_DIR}")
endif()

add_library(feedback_recorder STATIC
  src/feedback_recorder.cpp)
target_include_directories(feedback_recorder PUBLIC include)
target_link_libraries(feedback_recorder PUBLIC Threads::Threads)

add_executable(d1_feedback_probe
  src/d1_feedback_probe.cpp
  "${UNITREE_D1_SDK_ROOT}/src/msg/ArmString_.cpp"
  "${UNITREE_D1_SDK_ROOT}/src/msg/PubServoInfo_.cpp")
target_include_directories(d1_feedback_probe PRIVATE
  include
  "${UNITREE_D1_SDK_ROOT}/src"
  "${UNITREE_SDK2_ROOT}/include"
  "${UNITREE_SDK2_ROOT}/thirdparty/include"
  "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
target_link_libraries(d1_feedback_probe PRIVATE
  feedback_recorder
  "${UNITREE_SDK2_LIBRARY}"
  "${UNITREE_DDS_LIBRARY_DIR}/libddsc.so"
  "${UNITREE_DDS_LIBRARY_DIR}/libddscxx.so"
  Threads::Threads
  rt)
# 固定到 SDK 随附 DDS，避免由当前 LD_LIBRARY_PATH 偶然选中 Foxy 或系统版本。
set_target_properties(d1_feedback_probe PROPERTIES
  BUILD_RPATH "${UNITREE_DDS_LIBRARY_DIR}"
  INSTALL_RPATH "${UNITREE_DDS_LIBRARY_DIR}")
# 使用 DT_RPATH 而不是可被当前 LD_LIBRARY_PATH 抢占的 RUNPATH，确保 ddsc 与
# ddscxx 始终从 SDK 同一目录成对加载；不修改系统级 DDS。
target_link_options(d1_feedback_probe PRIVATE "-Wl,--disable-new-dtags")

if(BUILD_TESTING)
  add_executable(test_feedback_parser test/test_feedback_parser.cpp)
  target_link_libraries(test_feedback_parser PRIVATE feedback_recorder)
  target_include_directories(test_feedback_parser PRIVATE include)
  add_test(NAME feedback_parser COMMAND test_feedback_parser)
endif()
