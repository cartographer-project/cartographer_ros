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

#ifndef CARTOGRAPHER_ROS_ROBOT_OPTIONS_H_
#define CARTOGRAPHER_ROS_ROBOT_OPTIONS_H_

#include <string>

#include "cartographer/common/port.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/node_options.h"

namespace cartographer_ros {

struct RobotOptions {
  ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  string map_frame;
  string tracking_frame;
  string published_frame;
  string odom_frame;
  bool provide_odom_frame;
  bool use_odometry;
  bool use_laser_scan;
  bool use_multi_echo_laser_scan;
  int num_point_clouds;
};

RobotOptions CreateRobotOptionsFromNodeOptions(NodeOptions options);
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_ROBOT_OPTIONS_H_
