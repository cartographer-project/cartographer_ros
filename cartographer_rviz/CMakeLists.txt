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

cmake_minimum_required(VERSION 2.8)

project(cartographer_rviz)

set(PACKAGE_DEPENDENCIES
  cartographer_ros_msgs
  eigen_conversions
  roscpp
  rviz
)

set(CMAKE_CXX_FLAGS "-pthread -std=c++11 -Wreorder")

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -DNDEBUG")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(FATAL_ERROR "Cartographer is too slow to be useful in debug mode.")
else()
  message(FATAL_ERROR "Unknown CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    ${PACKAGE_DEPENDENCIES}
)

find_package(cartographer REQUIRED)
include_directories(${CARTOGRAPHER_INCLUDE_DIRS})
link_directories(${CARTOGRAPHER_LIBRARY_DIRS})

find_package(ZLIB REQUIRED)
include_directories(${ZLIB_INCLUDE_DIRS})

set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()
add_definitions(-DQT_NO_KEYWORDS)

find_package(Boost REQUIRED
 COMPONENTS
 system
 iostreams
)
add_definitions(${BOOST_DEFINITIONS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_library(cartographer_rviz_submaps_visualization
  src/drawable_submap.cc
  src/drawable_submap.h
  src/submaps_display.cc
  src/submaps_display.h
)
target_link_libraries(cartographer_rviz_submaps_visualization
  ${Boost_LIBRARIES}
  ${CARTOGRAPHER_LIBRARIES}
  ${QT_LIBRARIES}
  ${ZLIB_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(cartographer_rviz_submaps_visualization
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS cartographer_rviz_submaps_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
