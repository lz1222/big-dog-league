cmake_minimum_required(VERSION 3.5)
project(rk_go2_sdk_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

set(RK_UNITREE_SDK2_INSTALL_CANDIDATES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2_official/install"
  "$ENV{UNITREE_SDK2_PREFIX}"
  "/home/unitree/rk_inspection_ws/third_party/unitree_sdk2_official/install"
  "/home/unitree/unitree_sdk2/install"
  "/home/unitree/unitree_go2_sdk_test/unitree_sdk2/install"
)

set(RK_UNITREE_SDK2_PREFIX "")
set(RK_UNITREE_SDK2_RUNTIME_DIRS "")

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

install(PROGRAMS scripts/cmd_vel_udp_forwarder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(unitree_sdk2_FOUND)
  add_executable(go2_sdk_speed_sweep src/go2_sdk_speed_sweep.cpp)
  target_link_libraries(go2_sdk_speed_sweep unitree_sdk2)
  set_target_properties(go2_sdk_speed_sweep PROPERTIES
    BUILD_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
    INSTALL_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
  )
  add_executable(go2_sdk_motion_action
    src/go2_sdk_motion_action.cpp
  )
  target_link_libraries(go2_sdk_motion_action unitree_sdk2)
  set_target_properties(go2_sdk_motion_action PROPERTIES
    BUILD_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
    INSTALL_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
  )
  add_executable(go2_sdk_velocity_action
    src/go2_sdk_velocity_action.cpp
  )
  target_link_libraries(go2_sdk_velocity_action unitree_sdk2)
  set_target_properties(go2_sdk_velocity_action PROPERTIES
    BUILD_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
    INSTALL_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
  )
  install(TARGETS
    go2_sdk_speed_sweep
    go2_sdk_motion_action
    go2_sdk_velocity_action
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "unitree_sdk2 not found; SDK direct tools will not be built. "
    "Build/install third_party/unitree_sdk2 first if SDK direct tests are needed."
  )
endif()

ament_package()
