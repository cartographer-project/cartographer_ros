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

add_executable(cartographer_node
  node_main.cc)
target_include_directories(cartographer_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(cartographer_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_node
  "cartographer_ros_msgs"
  "geometry_msgs"
  "nav_msgs"
  "pcl_conversions"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_ros"
  "visualization_msgs"
)

add_executable(occupancy_grid_node
  occupancy_grid_node_main.cc)
target_include_directories(occupancy_grid_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(occupancy_grid_node ${PROJECT_NAME})
ament_target_dependencies(occupancy_grid_node
  "cartographer_ros_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
)

install(TARGETS
  cartographer_node
  occupancy_grid_node
  DESTINATION lib/${PROJECT_NAME})

# google_binary(cartographer_assets_writer
#   SRCS
#     assets_writer_main.cc
#     ros_map_writing_points_processor.h
#     ros_map_writing_points_processor.cc
# )
#
# install(TARGETS cartographer_assets_writer
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#
# google_binary(cartographer_offline_node
#   SRCS
#     offline_node_main.cc
# )
#
# install(TARGETS cartographer_offline_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# google_binary(cartographer_start_trajectory
#   SRCS
#     start_trajectory_main.cc
# )
#
# install(TARGETS cartographer_start_trajectory
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# google_binary(cartographer_occupancy_grid_node
#   SRCS
#     occupancy_grid_node_main.cc
# )
#
# install(TARGETS cartographer_occupancy_grid_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# google_binary(cartographer_rosbag_validate
#   SRCS
#     rosbag_validate_main.cc
# )
#
# install(TARGETS cartographer_rosbag_validate
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# google_binary(cartographer_pbstream_to_ros_map
#   SRCS
#     pbstream_to_ros_map_main.cc
# )
#
# install(TARGETS cartographer_pbstream_to_ros_map
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
