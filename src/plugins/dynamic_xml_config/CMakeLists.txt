# 安装场景生成入口及 plugins 资源目录，使用 ament_cmake 打包。
cmake_minimum_required(VERSION 3.8)
project(dynamic_xml_config)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  main_generate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins
)

# 生成 ament 包索引和配置导出，使安装后的包可被 ROS 2 构建与运行工具找到。
ament_package()
