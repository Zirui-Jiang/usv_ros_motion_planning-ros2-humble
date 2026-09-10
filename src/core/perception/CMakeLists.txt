cmake_minimum_required(VERSION 3.8)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(glog REQUIRED)

ament_export_dependencies(rclcpp common nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs glog)

include_directories(
  include
)

add_library(${PROJECT_NAME}_mapping SHARED
  src/mapping/mapping.cpp
)
ament_target_dependencies(${PROJECT_NAME}_mapping
  rclcpp
  common
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(${PROJECT_NAME}_mapping_node src/mapping/mapping_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapping_node
  rclcpp
  common
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_mapping_node
  ${PROJECT_NAME}_mapping
)

install(TARGETS ${PROJECT_NAME}_mapping
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/tf_process/usv_tf_expander.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
