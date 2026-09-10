cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common REQUIRED)

ament_export_dependencies(rclcpp nav2_costmap_2d geometry_msgs nav_msgs tf2 tf2_ros visualization_msgs common)

include_directories(
  include
)

file(GLOB path_planner_src_files src/*.cpp)
file(GLOB path_planner_utils_src_files src/utils/*.cpp)
file(GLOB graph_path_planner_src_files src/graph_planner/*.cpp)

add_library(${PROJECT_NAME} SHARED
  ${path_planner_src_files}
  ${path_planner_utils_src_files}
  ${graph_path_planner_src_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
  common
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
