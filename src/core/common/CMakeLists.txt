cmake_minimum_required(VERSION 3.8)
project(common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

ament_export_dependencies(rclcpp nav2_costmap_2d nav_msgs std_msgs visualization_msgs geometry_msgs tf2 tf2_ros Eigen3 glog)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

file(GLOB util_src_files src/util/*.cpp)
file(GLOB math_src_files_cpp src/math/*.cpp)
file(GLOB geometry_common_src_files src/geometry/*.cpp)
file(GLOB geometry_curve_src_files src/geometry/curve/*.cpp)

add_library(${PROJECT_NAME} SHARED
  ${util_src_files}
  ${math_src_files_cpp}
  ${geometry_common_src_files}
  ${geometry_curve_src_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  nav2_costmap_2d
  nav_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  glog::glog
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
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
