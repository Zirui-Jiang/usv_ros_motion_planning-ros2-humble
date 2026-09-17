cmake_minimum_required(VERSION 3.8)
project(usv_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wave_gazebo_plugins REQUIRED)

# Silence CMP0054 warnings coming from FindBoost.cmake used by gazebo.
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

ament_export_dependencies(rclcpp gazebo_dev gazebo_ros std_msgs sensor_msgs tf2 wave_gazebo_plugins)

set(GAZEBO_INCLUDE_DIRS_FULL
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-MATHS_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS_FULL}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-MATHS_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
)

########################################
# Libraries
########################################

add_library(buoyancy_gazebo_plugin SHARED
  src/buoyancy_gazebo_plugin.cc
  src/shape_volume.cc
  src/polyhedron_volume.cc
)
ament_target_dependencies(buoyancy_gazebo_plugin
  rclcpp gazebo_ros gazebo_dev std_msgs sensor_msgs tf2 wave_gazebo_plugins
)
target_link_libraries(buoyancy_gazebo_plugin
  ${GAZEBO_LIBRARIES}
  Eigen3::Eigen
)

add_library(usv_gazebo_dynamics_plugin SHARED
  src/usv_gazebo_dynamics_plugin.cc
)
ament_target_dependencies(usv_gazebo_dynamics_plugin
  rclcpp gazebo_ros gazebo_dev std_msgs sensor_msgs tf2 wave_gazebo_plugins
)
target_link_libraries(usv_gazebo_dynamics_plugin
  ${GAZEBO_LIBRARIES}
  Eigen3::Eigen
)

add_library(otter_usv_gazebo_thrust_plugin SHARED
  src/otter_usv_gazebo_thrust_plugin.cc
)
ament_target_dependencies(otter_usv_gazebo_thrust_plugin
  rclcpp gazebo_ros gazebo_dev std_msgs sensor_msgs tf2 wave_gazebo_plugins
)
target_link_libraries(otter_usv_gazebo_thrust_plugin
  ${GAZEBO_LIBRARIES}
  Eigen3::Eigen
)

add_library(usv_gazebo_wind_plugin SHARED
  src/usv_gazebo_wind_plugin.cc
)
ament_target_dependencies(usv_gazebo_wind_plugin
  rclcpp gazebo_ros gazebo_dev std_msgs sensor_msgs tf2 wave_gazebo_plugins
)
target_link_libraries(usv_gazebo_wind_plugin
  ${GAZEBO_LIBRARIES}
  Eigen3::Eigen
)

########################################
# Install
########################################

install(TARGETS
  buoyancy_gazebo_plugin
  usv_gazebo_dynamics_plugin
  otter_usv_gazebo_thrust_plugin
  usv_gazebo_wind_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
