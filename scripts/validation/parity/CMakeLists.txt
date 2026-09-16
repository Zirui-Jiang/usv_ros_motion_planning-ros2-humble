# 构建调用原 C++ 算法库的数值对照程序，供 Python/C++ 一致性检查使用。
cmake_minimum_required(VERSION 3.16)
project(usv_baseline_parity LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
# 查找构建所需依赖，名称与 package.xml 中的声明对应。
find_package(ament_cmake REQUIRED)
find_package(path_planner REQUIRED)
find_package(path_processor REQUIRED)
find_package(trajectory_planner REQUIRED)
find_package(controller REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(cpp_golden golden.cpp)
ament_target_dependencies(cpp_golden path_planner path_processor trajectory_planner controller)
target_link_libraries(cpp_golden yaml-cpp)
