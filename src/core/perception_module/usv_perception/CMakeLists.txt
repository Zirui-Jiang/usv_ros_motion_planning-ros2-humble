# 混合感知包构建：安装 Python 适配器和脚本，同时编译负责 Nav2 生命周期的 C++ 代价地图节点。
cmake_minimum_required(VERSION 3.8)
project(usv_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Threads REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/perception_mapping_node
  scripts/perception_monitor
  scripts/usv_tf_expander.py
  DESTINATION lib/${PROJECT_NAME})

# C++ 可执行程序只承接 Nav2 代价地图生命周期和线程管理，算法建图在 Python 包中。
add_executable(costmap_support_node src/costmap_support_node.cpp)
ament_target_dependencies(costmap_support_node
  rclcpp nav2_costmap_2d nav2_util lifecycle_msgs)
target_link_libraries(costmap_support_node Threads::Threads)

install(TARGETS costmap_support_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
