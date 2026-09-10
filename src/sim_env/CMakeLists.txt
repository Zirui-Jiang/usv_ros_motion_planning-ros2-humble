cmake_minimum_required(VERSION 3.8)
project(sim_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(path_planner REQUIRED)
find_package(trajectory_planner REQUIRED)
find_package(controller REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  nav2_costmap_2d nav2_util gazebo_ros path_planner trajectory_planner controller)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
  src/simulation.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  nav2_costmap_2d
  nav2_util
  path_planner
  trajectory_planner
  controller
)

add_executable(${PROJECT_NAME}_node
  src/simulation_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  nav2_costmap_2d
  nav2_util
  path_planner
  trajectory_planner
  controller
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

add_executable(fake_localization_node
  src/fake_localization_node.cpp
)

ament_target_dependencies(fake_localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Baseline validation: install the shared library required by sim_env_node.
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node fake_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz urdf worlds meshes models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
