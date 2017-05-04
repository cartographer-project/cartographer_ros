/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/robot_options.h"

#include "glog/logging.h"

namespace cartographer_ros {

RobotOptions CreateRobotOptionsFromNodeOptions(NodeOptions options) {
  RobotOptions robot_options;
  robot_options.map_builder_options = options.map_builder_options;
  robot_options.map_frame = options.map_frame;
  robot_options.tracking_frame = options.tracking_frame;
  robot_options.published_frame = options.published_frame;
  robot_options.odom_frame = options.odom_frame;
  robot_options.provide_odom_frame = options.provide_odom_frame;
  robot_options.use_odometry = options.use_odometry;
  robot_options.use_laser_scan = options.use_laser_scan;
  robot_options.use_multi_echo_laser_scan = options.use_multi_echo_laser_scan;
  robot_options.num_point_clouds = options.num_point_clouds;

  CHECK_EQ(robot_options.use_laser_scan +
           robot_options.use_multi_echo_laser_scan +
           (robot_options.num_point_clouds > 0),
           1)
      << "Configuration error: 'use_laser_scan', "
         "'use_multi_echo_laser_scan' and 'num_point_clouds' are "
         "mutually exclusive, but one is required.";

  if (robot_options.map_builder_options.use_trajectory_builder_2d()) {
    // Using point clouds is only supported in 3D.
    CHECK_EQ(options.num_point_clouds, 0);
  }
  return robot_options;
}
}  // namespace cartographer_ros
