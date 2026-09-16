# 将 RViz 生成脚本安装到 lib/包名，作为 ros2 run 可发现的程序。
cmake_minimum_required(VERSION 3.8)
project(dynamic_rviz_config)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/rviz_generate.py
  DESTINATION lib/${PROJECT_NAME}
)

# 生成 ament 包索引和配置导出，使安装后的包可被 ROS 2 构建与运行工具找到。
ament_package()
