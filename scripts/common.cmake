# =============================================================================
# TODO: Add file description
# =============================================================================
# License: MIT
# Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
# =============================================================================

# CMake Parameters and Variables
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Get all the cpp files in the src directory
file(GLOB SRCS ${SRC_DIR}/*.cpp ${SRC_DIR}/**/*.cpp)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${PROJECT_HOME}/scripts)

set(PROJECT_HOME ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# set(CMAKE_CXX_FLAGS "-Wall -Wno-ignored-qualifiers")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Spinnaker REQUIRED)

# header files and libraries
include_directories(
  ${INC_DIR} # project local include directory
  ${PROJECT_HOME}/lib/
  ${PROJECT_HOME}/assets/
  ${OpenCV_INCLUDE_DIRS}
  ${Spinnaker_INCLUDE_DIRS}
)

# Library linkables
link_directories(
  ${Spinnaker_LIB_DIRS}
  ${OpenCV_LIB_DIRS}
)

# Project Local Library
file(GLOB LIB_SRCS ${PROJECT_HOME}/lib/**/*.cpp)

# Get all the generates assets
file(GLOB ASSETS ${PROJECT_HOME}/assets/*.o)
SET_SOURCE_FILES_PROPERTIES(
  ${ASSETS}
  PROPERTIES
  EXTERNAL_OBJECT true
  GENERATED true
)

# Link with libraries
link_libraries(
  ${OpenCV_LIBS}
  ${Spinnaker_LIBRARIES}
)
