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

ament_package()
