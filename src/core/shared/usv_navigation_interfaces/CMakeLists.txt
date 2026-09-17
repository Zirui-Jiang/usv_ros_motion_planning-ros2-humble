# 生成四类导航 ROS 消息的语言绑定，并导出运行时依赖供决策、控制及感知包使用。
cmake_minimum_required(VERSION 3.8)
project(usv_navigation_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# 声明消息文件和被引用的标准头/二维目标位姿类型。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/RouteWaypoint.msg"
  "msg/NavigationReference.msg"
  "msg/ModuleHeartbeat.msg"
  "msg/ControlReport.msg"
  "srv/NavigateTo.srv"
  "srv/NavigateRoute.srv"
  "srv/StopNavigation.srv"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
