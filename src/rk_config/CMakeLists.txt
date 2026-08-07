cmake_minimum_required(VERSION 3.5)
project(rk_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
