# =============================================================================
# TODO: Add file description
# =============================================================================
# License: MIT
# Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
# =============================================================================


# FoveaCam CMake Recipe
cmake_minimum_required(VERSION 3.10)
project(spinnaker_camera)
set(PROJECT_HOME ${CMAKE_CURRENT_SOURCE_DIR}/..)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_HOME}/scripts)
include(common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(ROS2DEPS std_msgs sensor_msgs cv_bridge)
foreach(DEP IN ITEMS ${ROS2DEPS})
  find_package(${DEP} REQUIRED)
endforeach()

# Compile to binary
add_executable(capture ${SRCS} ${LIB_SRCS} ${ASSETS})
ament_target_dependencies(capture ${ROS2DEPS})
install(TARGETS capture DESTINATION lib/${PROJECT_NAME})

ament_package()
