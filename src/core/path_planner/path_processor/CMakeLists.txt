cmake_minimum_required(VERSION 3.8)
project(path_processor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)

ament_export_dependencies(rclcpp common)

include_directories(
  include
)

file(GLOB path_processor_src_fils src/path_processor/*.cpp)
file(GLOB path_prune_src_fils src/path_prune/*.cpp)
file(GLOB path_smooth_src_fils src/path_smooth/*.cpp)

add_library(${PROJECT_NAME} SHARED
  ${path_prune_src_fils}
  ${path_processor_src_fils}
  ${path_smooth_src_fils}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  common
)

install(DIRECTORY include/path_planner/
  DESTINATION include/path_planner
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
