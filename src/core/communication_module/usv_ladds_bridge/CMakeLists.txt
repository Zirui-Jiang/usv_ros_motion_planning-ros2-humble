cmake_minimum_required(VERSION 3.8)
project(usv_ladds_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/usv_ladds_bridge_node
  scripts/usv_ladds_route_node
  scripts/usv_ladds_radar_node
  scripts/ladds_single_leg_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_pytest test TIMEOUT 60)
endif()

ament_package()
