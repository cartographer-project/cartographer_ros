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
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

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

find_package(Eigen3 REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

include_directories(
  include
  ".")

file(GLOB_RECURSE ALL_SRCS "cartographer_rviz/*.cc" "cartographer_rviz/*.h")
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)
register_rviz_ogre_media_exports(DIRECTORIES "ogre_media/materials/glsl120" "ogre_media/materials/scripts")

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  cartographer
  cartographer_ros
  cartographer_ros_msgs
  rviz2
  rviz_common
  pcl_conversions
  rcutils
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY}
  ${cartographer_ros_LIBRARIES}
  -lstdc++fs
  -ldl
  -lboost_thread
  )
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

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

message(STATUS "Using Qt5 based on the rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
find_package(Qt5 ${rviz2_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)
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
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

install(FILES
  rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ogre_media"
  DESTINATION "share/${PROJECT_NAME}"
  )

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# replaces catkin_package(LIBRARIES ${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

#LD_LIBRARY_PATH()
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
