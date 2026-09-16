# 使用 ROS 接口生成器从 SetMission.srv 生成各语言服务类型，并导出运行时依赖。
cmake_minimum_required(VERSION 3.8)
project(usv_mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/SetMission.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
