cmake_minimum_required(VERSION 3.16)

project(go2_inspection_runner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(INSPECTION_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

add_library(inspection_core
  src/arm_controller_sim.cpp
  src/config.cpp
  src/logger.cpp
  src/mission_fsm.cpp
  src/robot_motion_sim.cpp
  src/vision_pipeline_sim.cpp
)

target_include_directories(inspection_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(go2_inspection_runner
  src/main.cpp
)

target_link_libraries(go2_inspection_runner
  PRIVATE inspection_core
)

if(MSVC)
  target_compile_options(inspection_core PRIVATE /W4)
  target_compile_options(go2_inspection_runner PRIVATE /W4)
  if(INSPECTION_WARNINGS_AS_ERRORS)
    target_compile_options(inspection_core PRIVATE /WX)
    target_compile_options(go2_inspection_runner PRIVATE /WX)
  endif()
else()
  target_compile_options(inspection_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(go2_inspection_runner PRIVATE -Wall -Wextra -Wpedantic)
  if(INSPECTION_WARNINGS_AS_ERRORS)
    target_compile_options(inspection_core PRIVATE -Werror)
    target_compile_options(go2_inspection_runner PRIVATE -Werror)
  endif()
endif()

