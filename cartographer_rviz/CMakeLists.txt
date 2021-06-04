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

project(cartographer_rviz)

find_package(ament_cmake REQUIRED)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()


find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
#find_package(rviz REQUIRED)

include_directories(
  include
  ".")

file(GLOB_RECURSE ALL_SRCS "cartographer_rviz/*.cc" "cartographer_rviz/*.h")
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} ${ALL_SRCS})
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  cartographer_ros
  cartographer_ros_msgs
  rviz2
  rviz_ogre_vendor
  rviz_assimp_vendor
  rviz_rendering
  rviz_common
  rviz_default_plugins

#  rviz
)

ament_export_dependencies(rosidl_default_runtime)

#if(rviz2_QT_VERSION VERSION_LESS "5")
#  message(STATUS "Using Qt4 based on the rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
  #find_package(Qt4 ${rviz2_QT_VERSION} EXACT REQUIRED QtCore QtGui)

#  include(${QT_USE_FILE})
#else()
  message(STATUS "Using Qt5 based on the rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
  find_package(Qt5 ${rviz2_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
  include_directories(${Qt5Widgets_INCLUDE_DIRS})
#endif()

add_definitions(-DQT_NO_KEYWORDS)
#add_library(${PROJECT_NAME} ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${QT_LIBRARIES})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${CARTOGRAPHER_ROS_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CARTOGRAPHER_ROS_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${RVIZ_OGRE_VENDOR_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${RVIZ_OGRE_VENDOR_LIBRARIES})

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)
target_link_libraries(${PROJECT_NAME} PUBLIC rviz_ogre_vendor)
target_link_libraries(${PROJECT_NAME} PUBLIC cartographer_ros)

install(DIRECTORY configuration_files
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY ogre_media
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
