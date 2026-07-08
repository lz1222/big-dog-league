cmake_minimum_required(VERSION 3.5)
project(rk_go2_sdk_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
set(RK_UNITREE_SDK2_INCLUDE_DIRS "")

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

function(rk_configure_unitree_sdk_target target_name)
  target_include_directories(${target_name} BEFORE PRIVATE
    ${RK_UNITREE_SDK2_INCLUDE_DIRS}
  )
  target_link_libraries(${target_name}
    unitree_sdk2
  )
  set_target_properties(${target_name} PROPERTIES
    BUILD_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
    INSTALL_RPATH "${RK_UNITREE_SDK2_RUNTIME_DIRS}"
  )
endfunction()

install(PROGRAMS
  scripts/cmd_vel_udp_forwarder.py
  scripts/warning_sign_image_classifier.py
  scripts/go2_warning_sign_sdk_loop.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(unitree_sdk2_FOUND)
  add_executable(go2_sdk_capture_image
    src/go2_sdk_capture_image.cpp
  )
  rk_configure_unitree_sdk_target(go2_sdk_capture_image)
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
  install(TARGETS
    go2_sdk_capture_image
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
