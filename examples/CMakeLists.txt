# =============================================================================
# TODO: Add file description
# =============================================================================
# License: MIT
# Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
# =============================================================================

cmake_minimum_required(VERSION 3.10)
project(spinnaker_camera_examples)
set(PROJECT_HOME ${CMAKE_CURRENT_SOURCE_DIR}/..)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_HOME}/scripts)
include(common)

# Output binaries to bin directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_HOME}/bin/examples)

# Compile to binary
foreach(SRC ${SRCS})
  get_filename_component(target ${SRC} NAME)
  string(REPLACE ".cpp" "" target ${target})
  add_executable(${target} ${SRC} ${LIB_SRCS} ${ASSETS})
endforeach()
