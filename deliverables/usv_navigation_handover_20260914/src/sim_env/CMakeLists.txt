cmake_minimum_required(VERSION 3.8)
project(sim_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Simulation truth adapter and compatibility Python entry.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/sim_env_node DESTINATION lib/${PROJECT_NAME})

add_executable(fake_localization_node src/fake_localization_node.cpp)
ament_target_dependencies(fake_localization_node
  rclcpp geometry_msgs nav_msgs gazebo_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS fake_localization_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz urdf worlds meshes models
  DESTINATION share/${PROJECT_NAME})

# gazebo_ros replaces the resource search path even when the user did not
# source Gazebo's setup.sh. Keep the installed RTShaderSystem library visible.
get_filename_component(GAZEBO_RESOURCE_ROOT "${GAZEBO_MEDIA_PATH}" DIRECTORY)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.sh.in")

ament_package()
