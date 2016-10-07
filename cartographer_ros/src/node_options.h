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

#ifndef CARTOGRAPHER_ROS_NODE_OPTIONS_H_
#define CARTOGRAPHER_ROS_NODE_OPTIONS_H_

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/map_builder.h"

#include "sensor_bridge.h"

namespace cartographer_ros {

// Top-level options of Cartographer's ROS integration.
struct NodeOptions {
  ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  SensorBridgeOptions sensor_bridge_options;
  string map_frame;
  string tracking_frame;
  string published_frame;
  string odom_frame;
  bool provide_odom_frame;
  bool use_odometry_data;
  bool use_horizontal_laser;
  bool use_horizontal_multi_echo_laser;
  int num_lasers_3d;
  double lookup_transform_timeout_sec;
  double submap_publish_period_sec;
  double pose_publish_period_sec;
};

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_NODE_OPTIONS_H_
