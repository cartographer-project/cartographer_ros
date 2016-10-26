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

find_package(Cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()

find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    ${PACKAGE_DEPENDENCIES}
  INCLUDE_DIRS ${PROJECT_NAME}
)

set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include_directories(${Qt4_INCLUDE_DIRS})
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  find_package(Qt5Widgets REQUIRED)
  set(QT_LIBRARIES Qt5::Widgets)
  include_directories(${Qt5Widgets_INCLUDE_DIRS})
  include_directories(${Qt5_INCLUDE_DIRS})
endif()
add_definitions(-DQT_NO_KEYWORDS)

SET(ALL_LIBRARIES "" CACHE INTERNAL "ALL_LIBRARIES")

add_subdirectory("cartographer_rviz")

# Create an empty file for the shared library we require for RViz.
set(DUMMY_SOURCE ${CMAKE_CURRENT_BINARY_DIR}/empty.cc)
add_custom_command(
  OUTPUT  ${DUMMY_SOURCE}
  COMMAND cmake -E touch ${DUMMY_SOURCE}
  DEPENDS ${ARG_SRCS}
)
add_library(cartographer_rviz_submaps_visualization ${DUMMY_SOURCE})
target_link_libraries(cartographer_rviz_submaps_visualization
  ${QT_LIBRARIES}
  "-Wl,--whole-archive"
  ${ALL_LIBRARIES}
  "-Wl,--no-whole-archive"
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
