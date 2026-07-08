cmake_minimum_required(VERSION 3.5)
project(unitree_d1_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(UNITREE_SDK2_SOURCE_CANDIDATES
  "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_sdk2"
  "/home/unitree/big-dog-league/third_party/unitree_sdk2"
  "/home/unitree/rk_inspection_ws/third_party/unitree_sdk2"
)

foreach(candidate ${UNITREE_SDK2_SOURCE_CANDIDATES})
  if(candidate AND
      EXISTS "${candidate}/CMakeLists.txt" AND
      EXISTS "${candidate}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a")
    set(BUILD_EXAMPLES OFF CACHE BOOL
      "Do not build unitree_sdk2 examples from unitree_d1_sdk" FORCE)
    add_subdirectory(
      "${candidate}"
      "${CMAKE_CURRENT_BINARY_DIR}/unitree_sdk2"
      EXCLUDE_FROM_ALL
    )
    set(UNITREE_SDK2_PREFIX "${candidate}")
    set(UNITREE_SDK2_RUNTIME_DIR
      "${candidate}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
    )
    set(unitree_sdk2_FOUND TRUE)
    message(STATUS
      "Using architecture-matched unitree_sdk2 source at ${candidate} "
      "for ${CMAKE_SYSTEM_PROCESSOR}"
    )
    break()
  endif()
endforeach()

set(UNITREE_SDK2_PREFIX_CANDIDATES
  "$ENV{UNITREE_SDK2_PREFIX}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_sdk2/install"
  "/home/unitree/rk_inspection_ws/third_party/unitree_sdk2/install"
  "/home/unitree/unitree_sdk2/install"
  "/opt/unitree_robotics"
  "/usr/local"
)

if(NOT unitree_sdk2_FOUND)
  foreach(candidate ${UNITREE_SDK2_PREFIX_CANDIDATES})
    if(candidate AND
        EXISTS "${candidate}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake")
      list(PREPEND CMAKE_PREFIX_PATH "${candidate}")
      set(UNITREE_SDK2_PREFIX "${candidate}")
      break()
    endif()
  endforeach()

  find_package(unitree_sdk2 QUIET CONFIG)
endif()

if(unitree_sdk2_FOUND)
  message(STATUS "Using unitree_sdk2 from ${UNITREE_SDK2_PREFIX}")
  set(D1_SDK_LIBRARIES unitree_sdk2 rt Threads::Threads)
else()
  message(WARNING
    "unitree_sdk2 CMake package was not found. Falling back to system "
    "include/link paths; set UNITREE_SDK2_PREFIX for a clean Ubuntu build."
  )
  include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
  set(D1_SDK_LIBRARIES unitree_sdk2 ddsc ddscxx rt Threads::Threads)
endif()

function(configure_d1_target target_name)
  target_link_libraries(${target_name} ${D1_SDK_LIBRARIES})
  if(UNITREE_SDK2_PREFIX)
    set_target_properties(${target_name} PROPERTIES
      BUILD_RPATH "${UNITREE_SDK2_PREFIX}/lib;${UNITREE_SDK2_RUNTIME_DIR}"
      INSTALL_RPATH "${UNITREE_SDK2_PREFIX}/lib;${UNITREE_SDK2_RUNTIME_DIR}"
    )
  endif()
endfunction()

# 编译成共享库
# add_library(grasp SHARED src/grasp.cpp src/msg/ArmString_.cpp)
add_library(left_1 SHARED src/left_1.cpp src/msg/ArmString_.cpp)
configure_d1_target(left_1)
add_library(left_2 SHARED src/left_2.cpp src/msg/ArmString_.cpp)
configure_d1_target(left_2)
add_library(right_1 SHARED src/right_1.cpp src/msg/ArmString_.cpp)
configure_d1_target(right_1)
add_library(right_2 SHARED src/right_2.cpp src/msg/ArmString_.cpp)
configure_d1_target(right_2)
add_library(grasp_1_1 SHARED src/grasp_1_1.cpp src/msg/ArmString_.cpp)
configure_d1_target(grasp_1_1)
add_library(grasp_1_2 SHARED src/grasp_1_2.cpp src/msg/ArmString_.cpp)
configure_d1_target(grasp_1_2)
add_library(grasp_2_1 SHARED src/grasp_2_1.cpp src/msg/ArmString_.cpp)
configure_d1_target(grasp_2_1)
add_library(grasp_2_2 SHARED src/grasp_2_2.cpp src/msg/ArmString_.cpp)
configure_d1_target(grasp_2_2)
add_library(grasp_3_1 SHARED src/grasp_3_1.cpp src/msg/ArmString_.cpp)
configure_d1_target(grasp_3_1)
add_library(grasp_3_2 SHARED src/grasp_3_2.cpp src/msg/ArmString_.cpp)
configure_d1_target(grasp_3_2)
add_library(grasp_4_1 SHARED src/grasp_4_1.cpp src/msg/ArmString_.cpp)
configure_d1_target(grasp_4_1)
add_library(grasp_4_2 SHARED src/grasp_4_2.cpp src/msg/ArmString_.cpp)
configure_d1_target(grasp_4_2)


add_executable(joint_angle_control src/joint_angle_control.cpp src/msg/ArmString_.cpp)
configure_d1_target(joint_angle_control)
add_executable(multiple_joint_angle_control src/multiple_joint_angle_control.cpp src/msg/ArmString_.cpp)
configure_d1_target(multiple_joint_angle_control)
add_executable(joint_enable_control src/joint_enable_control.cpp src/msg/ArmString_.cpp)
configure_d1_target(joint_enable_control)
add_executable(arm_zero_control src/arm_zero_control.cpp src/msg/ArmString_.cpp)
configure_d1_target(arm_zero_control)
add_executable(get_arm_joint_angle src/get_arm_joint_angle.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
configure_d1_target(get_arm_joint_angle)
add_executable(d1_servo_heat_diagnosis src/d1_servo_heat_diagnosis.cpp src/msg/ArmString_.cpp src/msg/PubServoInfo_.cpp)
configure_d1_target(d1_servo_heat_diagnosis)
