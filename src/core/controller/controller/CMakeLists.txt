cmake_minimum_required(VERSION 3.8)
project(controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(trajectory_planner REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(rclcpp common trajectory_planner nav2_costmap_2d tf2 tf2_ros visualization_msgs Eigen3)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

file(GLOB controller_src_files src/*.cpp)
file(GLOB pid_controller_src_files src/pid_controller/*.cpp)
file(GLOB lqr_controller_src_files src/lqr_controller/*.cpp)

add_library(${PROJECT_NAME} SHARED
  ${controller_src_files}
  ${pid_controller_src_files}
  ${lqr_controller_src_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  common
  trajectory_planner
  nav2_costmap_2d
  tf2
  tf2_ros
  visualization_msgs
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
