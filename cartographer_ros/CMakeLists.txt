# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(cartographer_ros)

find_package(ament_cmake REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)

google_initialize_cartographer_project()
# google_enable_testing()

find_package(cartographer_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
# find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

include(FindPkgConfig)

find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(urdfdom_headers REQUIRED)

if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  include
  "."
  ${cartographer_INCLUDE_DIRS}
  ${cartographer_ros_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# # Override Catkin's GTest configuration to use GMock.
# set(GTEST_FOUND TRUE)
# set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
# set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

# file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h")
# file(GLOB_RECURSE ALL_TESTS "*_test.cc")
# file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")
# list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
# list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})

set(ALL_SRCS
  "cartographer_ros/map_builder_bridge.cc"
  "cartographer_ros/msg_conversion.cc"
  "cartographer_ros/node.cc"
  "cartographer_ros/node_constants.cc"
  "cartographer_ros/node_options.cc"
  "cartographer_ros/ros_log_sink.cc"
  "cartographer_ros/sensor_bridge.cc"
  "cartographer_ros/tf_bridge.cc"
  "cartographer_ros/time_conversion.cc"
  "cartographer_ros/trajectory_options.cc"
  "cartographer_ros/submap.cc"
  )
add_library(${PROJECT_NAME} ${ALL_SRCS})
add_subdirectory("cartographer_ros")

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

# # Boost
# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
#   "${Boost_INCLUDE_DIRS}")
# target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# YAML
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAMLCPP_LIBRARIES})

# URDFDOM
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${urdfdom_headers_INCLUDE_DIRS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

# install(PROGRAMS scripts/tf_remove_frames.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY configuration_files
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# # Install source headers.
# file(GLOB_RECURSE HDRS "*.h")
# foreach(HDR ${HDRS})
#   file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
#   get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
#   install(
#     FILES
#       ${HDR}
#     DESTINATION
#       include/${INSTALL_DIR}
#   )
# endforeach()

ament_package()
