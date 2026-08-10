cmake_minimum_required(VERSION 3.5)
project(rk_go2_sdk_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

add_library(rk_udp_motion_core STATIC
  src/udp_motion_core.cpp
)
target_include_directories(rk_udp_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(RK_UNITREE_SDK2_INSTALL_CANDIDATES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2_official/install"
  "$ENV{UNITREE_SDK2_PREFIX}"
  "/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/install"
  "/home/unitree/unitree_sdk2/install"
  "/home/unitree/unitree_go2_sdk_test/unitree_sdk2/install"
)

set(RK_UNITREE_SDK2_PREFIX "")
set(RK_UNITREE_SDK2_RUNTIME_DIRS "")
set(RK_UNITREE_SDK2_INCLUDE_DIRS "")
set(RK_UNITREE_DDS_RUNTIME_INSTALL_DIR "unitree_sdk_runtime")

foreach(candidate ${RK_UNITREE_SDK2_INSTALL_CANDIDATES})
  if(candidate AND
      EXISTS "${candidate}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake")
    set(RK_UNITREE_SDK2_PREFIX "${candidate}")
    list(PREPEND CMAKE_PREFIX_PATH "${candidate}")
    break()
  endif()
endforeach()

if(RK_UNITREE_SDK2_PREFIX)
  find_package(unitree_sdk2 QUIET CONFIG
    PATHS "${RK_UNITREE_SDK2_PREFIX}"
    NO_DEFAULT_PATH
  )
endif()

if(unitree_sdk2_FOUND)
  list(APPEND RK_UNITREE_SDK2_RUNTIME_DIRS "${RK_UNITREE_SDK2_PREFIX}/lib")
else()
  set(RK_UNITREE_SDK2_SOURCE_CANDIDATES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2_official"
    "/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2"
    "/home/unitree/rk_inspection_ws/third_party/unitree_sdk2"
  )

  foreach(candidate ${RK_UNITREE_SDK2_SOURCE_CANDIDATES})
    if(candidate AND
        EXISTS "${candidate}/CMakeLists.txt" AND
        EXISTS "${candidate}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a")
      set(BUILD_EXAMPLES OFF CACHE BOOL
        "Do not build unitree_sdk2 examples from rk_go2_sdk_bridge" FORCE)
      add_subdirectory(
        "${candidate}"
        "${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2"
        EXCLUDE_FROM_ALL
      )
      set(RK_UNITREE_SDK2_PREFIX "${candidate}")
      list(APPEND RK_UNITREE_SDK2_RUNTIME_DIRS
        "${candidate}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
      )
      set(unitree_sdk2_FOUND TRUE)
      message(STATUS
        "Using unitree_sdk2 source at ${candidate} for "
        "${CMAKE_SYSTEM_PROCESSOR}"
      )
      break()
    endif()
  endforeach()
endif()

foreach(include_candidate
    "${RK_UNITREE_SDK2_PREFIX}/include"
    "${RK_UNITREE_SDK2_PREFIX}/thirdparty/include"
    "${RK_UNITREE_SDK2_PREFIX}/../thirdparty/include")
  if(include_candidate AND EXISTS "${include_candidate}")
    list(APPEND RK_UNITREE_SDK2_INCLUDE_DIRS "${include_candidate}")
  endif()
endforeach()
list(REMOVE_DUPLICATES RK_UNITREE_SDK2_INCLUDE_DIRS)

# SDK 进程必须携带同一对 CycloneDDS 库，不能依赖开发机源码树或 ROS shell。
# 配置仅保存相对安装目录，wrapper 会相对自身安装位置解析它。
if(RK_UNITREE_SDK2_RUNTIME_DIRS)
  list(GET RK_UNITREE_SDK2_RUNTIME_DIRS 0 RK_UNITREE_SDK2_RUNTIME_DIR)
  set(RK_UNITREE_SDK2_RUNTIME_RELATIVE_DIR
    "${RK_UNITREE_DDS_RUNTIME_INSTALL_DIR}")
  configure_file(
    scripts/go2_sdk_runtime_dir.txt.in
    "${CMAKE_CURRENT_BINARY_DIR}/go2_sdk_runtime_dir.txt"
    @ONLY
  )
endif()

function(rk_configure_unitree_sdk_target target_name)
  target_include_directories(${target_name} BEFORE PRIVATE
    ${RK_UNITREE_SDK2_INCLUDE_DIRS}
  )
  target_link_libraries(${target_name}
    unitree_sdk2
  )
  set_target_properties(${target_name} PROPERTIES
    BUILD_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
    INSTALL_RPATH "\$ORIGIN/${RK_UNITREE_DDS_RUNTIME_INSTALL_DIR}"
  )
  # DT_RPATH 优先于父 LD_LIBRARY_PATH，阻止所有 SDK 工具混入 ROS DDS 库。
  set_property(TARGET ${target_name} APPEND_STRING
    PROPERTY LINK_FLAGS " -Wl,--disable-new-dtags")
endfunction()

install(PROGRAMS
  scripts/cmd_vel_udp_forwarder.py
  scripts/sdk_motion_status.py
  scripts/sdk_motion_status_gate.py
  scripts/sdk_motion_status_audit.py
  scripts/warning_sign_image_classifier.py
  scripts/go2_warning_sign_sdk_loop.sh
  scripts/go2_front_camera_bridge.py
  scripts/go2_sdk_server_runtime.py
  scripts/go2_control_plane_gate.py
  scripts/go2_control_plane_measure.py
  scripts/stair_camera_gait_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

if(RK_UNITREE_SDK2_RUNTIME_DIRS)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/go2_sdk_runtime_dir.txt"
    DESTINATION lib/${PROJECT_NAME}
  )
  # 保留 SONAME 名称和开发链接名，确保安装树可脱离源码树独立解析。
  install(FILES
    "${RK_UNITREE_SDK2_RUNTIME_DIR}/libddsc.so"
    "${RK_UNITREE_SDK2_RUNTIME_DIR}/libddscxx.so"
    DESTINATION lib/${PROJECT_NAME}/${RK_UNITREE_DDS_RUNTIME_INSTALL_DIR}
  )
  install(FILES "${RK_UNITREE_SDK2_RUNTIME_DIR}/libddsc.so"
    DESTINATION lib/${PROJECT_NAME}/${RK_UNITREE_DDS_RUNTIME_INSTALL_DIR}
    RENAME libddsc.so.0
  )
  install(FILES "${RK_UNITREE_SDK2_RUNTIME_DIR}/libddscxx.so"
    DESTINATION lib/${PROJECT_NAME}/${RK_UNITREE_DDS_RUNTIME_INSTALL_DIR}
    RENAME libddscxx.so.0
  )
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(unitree_sdk2_FOUND)
  add_executable(go2_sdk_udp_server
    src/go2_sdk_udp_server.cpp
  )
  target_link_libraries(go2_sdk_udp_server
    rk_udp_motion_core
  )
  rk_configure_unitree_sdk_target(go2_sdk_udp_server)
  add_executable(go2_sdk_udp_server_v2
    src/go2_sdk_udp_server_v2.cpp
  )
  target_link_libraries(go2_sdk_udp_server_v2
    rk_udp_motion_core
  )
  rk_configure_unitree_sdk_target(go2_sdk_udp_server_v2)
  add_executable(go2_sdk_capture_image
    src/go2_sdk_capture_image.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_capture_image)
  add_executable(go2_front_camera_stream_helper
    src/go2_front_camera_stream_helper.cpp
  )
  target_include_directories(go2_front_camera_stream_helper PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  rk_configure_unitree_sdk_target(go2_front_camera_stream_helper)
  add_executable(go2_sdk_speed_sweep src/go2_sdk_speed_sweep.cpp)
  rk_configure_unitree_sdk_target(go2_sdk_speed_sweep)
  add_executable(go2_sdk_motion_action
    src/go2_sdk_motion_action.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_motion_action)
  add_executable(go2_sdk_velocity_action
    src/go2_sdk_velocity_action.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_velocity_action)
  # 楼梯入口专用经典步态工具；不复用迷宫路线或其速度/转弯参数。
  add_executable(go2_sdk_stair_classic_action
    src/go2_sdk_stair_classic_action.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_stair_classic_action)
  # 平地分级减速验证；与楼梯完整流程完全独立，默认不执行真机动作。
  add_executable(go2_sdk_flat_deceleration_test
    src/go2_sdk_flat_deceleration_test.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_flat_deceleration_test)
  # 独立经典步态直行验证工具，默认 Dry Run，显式 --execute 才允许调用 SDK。
  add_executable(go2_sdk_classic_walk_probe
    src/go2_sdk_classic_walk_probe.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_classic_walk_probe)
  # 独立构建已冻结的写死迷宫路线快照，避免后续调试改动影响该可复现实验版本。
  add_executable(go2_sdk_maze_hardcoded_v1
    ${CMAKE_CURRENT_SOURCE_DIR}/../测试代码/迷宫写死完美通过第一版.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_maze_hardcoded_v1)
  add_executable(go2_sdk_sport_state_monitor
    src/go2_sdk_sport_state_monitor.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_sport_state_monitor)
  # 逐字段保存完整 SportModeState；仅订阅，供步态签名离线比较。
  add_executable(go2_sdk_sport_state_raw_capture
    src/go2_sdk_sport_state_raw_capture.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_sport_state_raw_capture)
  # 被动记录 APP/遥控器可能下发的高层 DDS 运动命令；不创建控制客户端。
  add_executable(go2_sdk_sport_mode_command_observer
    src/go2_sdk_sport_mode_command_observer.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_sport_mode_command_observer)
  # 被动记录遥控器按键边沿，供人工步态切换路径取证；不包含控制发布者。
  add_executable(go2_sdk_wireless_controller_observer
    src/go2_sdk_wireless_controller_observer.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_wireless_controller_observer)
  # 只读 RobotState RPC 诊断，禁止在此工具中引入任何动作客户端。
  add_executable(go2_sdk_robot_state_probe
    src/go2_sdk_robot_state_probe.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_robot_state_probe)
  # 仅用于 Sport RPC 版本握手诊断；不包含任何动作调用。
  add_executable(go2_sdk_sport_identity_probe
    src/go2_sdk_sport_identity_probe.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_sport_identity_probe)
  # AutoRecoverGet 是唯一允许的非运动 Sport Call，用于诊断 RPC 路径。
  add_executable(go2_sdk_sport_autorecover_get_probe
    src/go2_sdk_sport_autorecover_get_probe.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_sport_autorecover_get_probe)
  # 被动订阅 Sport API request/response；可用过滤或全量观察，不创建 SDK Client。
  add_executable(go2_sdk_sport_api_observer
    src/go2_sdk_sport_api_observer.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_sport_api_observer)
  # 单次 StopMove 仅供被动 DDS trace 关联，严禁在此工具中增加重试。
  add_executable(go2_sdk_stopmove_trace_probe
    src/go2_sdk_stopmove_trace_probe.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_stopmove_trace_probe)
  # 独立 MotionSwitcher 只读 CheckMode probe。
  add_executable(go2_sdk_motion_switcher_check_mode_probe
    src/go2_sdk_motion_switcher_check_mode_probe.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_motion_switcher_check_mode_probe)
  # 人工安全前提成立后使用的单次模式恢复步骤；不属于常规运动链路。
  add_executable(go2_sdk_release_mode_once
    src/go2_sdk_release_mode_once.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_release_mode_once)
  add_executable(go2_sdk_sport_mode_switch_once
    src/go2_sdk_sport_mode_switch_once.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_sport_mode_switch_once)
  # 正式启动前唯一的 motion ownership / sport responder 恢复门。
  add_executable(go2_motion_control_prearm
    src/go2_motion_control_prearm.cpp
  )
  rk_configure_unitree_sdk_target(go2_motion_control_prearm)
  # 固定 normal 的一次性 mode 选择，避免任意模式名进入恢复流程。
  add_executable(go2_sdk_select_normal_mode_once
    src/go2_sdk_select_normal_mode_once.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_select_normal_mode_once)
  # 省赛合同的唯一经典步态启动 backend；默认 dry-run，显式参数才调用 SDK。
  add_executable(go2_sdk_competition_gait_manager
    src/go2_sdk_competition_gait_manager.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_competition_gait_manager)
  add_executable(fake_sdk_motion_helper
    src/fake_sdk_motion_helper.cpp
  )
  install(TARGETS
    go2_sdk_udp_server
    go2_sdk_udp_server_v2
    go2_sdk_capture_image
    go2_sdk_speed_sweep
    go2_sdk_motion_action
    go2_sdk_velocity_action
    go2_sdk_stair_classic_action
    go2_sdk_flat_deceleration_test
    go2_sdk_classic_walk_probe
    go2_sdk_maze_hardcoded_v1
    go2_sdk_sport_state_monitor
    go2_sdk_sport_state_raw_capture
    go2_sdk_sport_mode_command_observer
    go2_sdk_wireless_controller_observer
    go2_sdk_robot_state_probe
    go2_sdk_sport_identity_probe
    go2_sdk_sport_autorecover_get_probe
    go2_sdk_sport_api_observer
    go2_sdk_stopmove_trace_probe
    go2_sdk_motion_switcher_check_mode_probe
    go2_sdk_release_mode_once
    go2_sdk_sport_mode_switch_once
    go2_motion_control_prearm
    go2_sdk_select_normal_mode_once
    go2_sdk_competition_gait_manager
    go2_front_camera_stream_helper
    fake_sdk_motion_helper
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "unitree_sdk2 not found; SDK direct tools will not be built. "
    "Build/install third_party/unitree_sdk2 first if SDK direct tests are needed."
  )
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_jpeg_frame_validation
    test/test_jpeg_frame_validation.cpp
  )
  target_include_directories(test_jpeg_frame_validation PRIVATE include)
  add_test(
    NAME test_jpeg_frame_validation
    COMMAND test_jpeg_frame_validation
  )
  add_executable(test_udp_motion_core
    test/test_udp_motion_core.cpp
  )
  target_link_libraries(test_udp_motion_core
    rk_udp_motion_core
  )
  add_test(
    NAME test_udp_motion_core
    COMMAND test_udp_motion_core
  )
  add_executable(test_motion_status_protocol
    test/test_motion_status_protocol.cpp
  )
  target_include_directories(test_motion_status_protocol PRIVATE include)
  target_link_libraries(test_motion_status_protocol rk_udp_motion_core)
  add_test(
    NAME test_motion_status_protocol
    COMMAND test_motion_status_protocol
  )
  add_executable(test_startup_stop_retry_core
    test/test_startup_stop_retry_core.cpp
  )
  target_include_directories(test_startup_stop_retry_core PRIVATE include)
  add_test(
    NAME test_startup_stop_retry_core
    COMMAND test_startup_stop_retry_core
  )
  add_test(
    NAME test_front_camera_bridge
    COMMAND python3 -m pytest
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_front_camera_bridge.py"
      -v --tb=short
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(
    NAME test_sdk_server_runtime
    COMMAND python3 -m pytest
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sdk_server_runtime.py"
      -v --tb=short
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(
    NAME test_sdk_motion_status
    COMMAND python3 -m pytest
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sdk_motion_status.py"
      -v --tb=short
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(
    NAME test_sdk_motion_status_gate
    COMMAND python3 -m pytest
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sdk_motion_status_gate.py"
      -v --tb=short
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(
    NAME test_sdk_motion_status_audit
    COMMAND python3 -m pytest
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sdk_motion_status_audit.py"
      -v --tb=short
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(
    NAME test_control_plane_gate
    COMMAND python3 -m pytest
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_control_plane_gate.py"
      -v --tb=short
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(
    NAME test_control_plane_measure
    COMMAND python3 -m pytest
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_control_plane_measure.py"
      -v --tb=short
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
