cmake_minimum_required(VERSION 3.8)
project(wave_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Silence CMP0054 warnings coming from FindBoost.cmake used by gazebo.
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(gazebo_ros gazebo_dev Eigen3)

set(GAZEBO_INCLUDE_DIRS_FULL
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-MATHS_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS_FULL}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-MATHS_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
)

###############################################################################
# Libraries...

# Hydrodynamics
add_library(Hydrodynamics SHARED
  src/Gazebo.cc
  src/Geometry.cc
  src/PhysicalConstants.cc
  src/Physics.cc
  src/Utilities.cc
  src/Wavefield.cc
  src/WavefieldEntity.cc
)
target_link_libraries(Hydrodynamics
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_compile_options(Hydrodynamics PRIVATE "-Wno-unknown-pragmas")

list(APPEND WAVE_GAZEBO_LIBRARIES_LIST Hydrodynamics)

###############################################################################
# Plugins...

# WavefieldModelPlugin
add_library(WavefieldModelPlugin SHARED
  src/WavefieldModelPlugin.cc
)
target_link_libraries(WavefieldModelPlugin
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${WAVE_GAZEBO_LIBRARIES_LIST}
)
target_compile_options(WavefieldModelPlugin PRIVATE "-Wno-unknown-pragmas")
list(APPEND WAVE_GAZEBO_PLUGINS_LIST WavefieldModelPlugin)

# WavefieldVisualPlugin
add_library(WavefieldVisualPlugin SHARED
  src/WavefieldVisualPlugin.cc
)
target_link_libraries(WavefieldVisualPlugin
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${WAVE_GAZEBO_LIBRARIES_LIST}
)
target_compile_options(WavefieldVisualPlugin PRIVATE "-Wno-unknown-pragmas")
list(APPEND WAVE_GAZEBO_PLUGINS_LIST WavefieldVisualPlugin)

# WavegaugePlugin
add_library(wavegauge_plugin SHARED
  src/wavegauge_plugin.cc
)
target_link_libraries(wavegauge_plugin
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${WAVE_GAZEBO_LIBRARIES_LIST}
)
add_dependencies(wavegauge_plugin WavefieldModelPlugin)
target_compile_options(wavegauge_plugin PRIVATE "-Wno-unknown-pragmas")
list(APPEND WAVE_GAZEBO_PLUGINS_LIST wavegauge_plugin)

###############################################################################
# Install

install(TARGETS ${WAVE_GAZEBO_LIBRARIES_LIST}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${WAVE_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(${WAVE_GAZEBO_LIBRARIES_LIST})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
