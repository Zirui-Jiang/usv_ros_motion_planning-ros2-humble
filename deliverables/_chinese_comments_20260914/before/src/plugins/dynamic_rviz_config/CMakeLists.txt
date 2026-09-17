cmake_minimum_required(VERSION 3.8)
project(dynamic_rviz_config)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/rviz_generate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
