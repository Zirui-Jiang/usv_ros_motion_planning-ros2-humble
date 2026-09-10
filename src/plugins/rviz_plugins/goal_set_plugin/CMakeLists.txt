cmake_minimum_required(VERSION 3.8)
project(goal_set_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

set(HEADER_FILES
  include/rviz_plugins/goal_set_plugin/goal_set_plugin.h
)

set(SOURCE_FILES
  src/goal_set_plugin.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
  ${HEADER_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  pluginlib
  rviz_common
  rviz_default_plugins
  rviz_rendering
  std_msgs
  tf2
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# Register the tool with pluginlib so rviz_common can discover the class at
# runtime. Without this, rviz reports the class does not exist even though the
# shared library builds and installs correctly.
pluginlib_export_plugin_description_file(rviz_common goal_set_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  goal_set_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rviz_common rviz_default_plugins)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
