cmake_minimum_required(VERSION 3.5)
project(rk_go2_sdk_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

set(RK_UNITREE_SDK2_PREFIX
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_sdk2/install"
)
if(EXISTS "${RK_UNITREE_SDK2_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${RK_UNITREE_SDK2_PREFIX}")
endif()

find_package(unitree_sdk2 QUIET)

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
    BUILD_RPATH "${RK_UNITREE_SDK2_PREFIX}/lib"
    INSTALL_RPATH "${RK_UNITREE_SDK2_PREFIX}/lib"
  )
  install(TARGETS go2_sdk_speed_sweep
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "unitree_sdk2 not found; go2_sdk_speed_sweep will not be built. "
    "Build/install third_party/unitree_sdk2 first if SDK direct tests are needed."
  )
endif()

ament_package()
