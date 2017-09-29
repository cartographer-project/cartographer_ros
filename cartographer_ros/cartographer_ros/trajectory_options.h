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

#ifndef CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_
#define CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"

namespace cartographer_ros {

struct TrajectoryOptions {
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  string tracking_frame;
  string published_frame;
  string odom_frame;
  bool provide_odom_frame;
  bool use_odometry;
  int num_laser_scans;
  int num_multi_echo_laser_scans;
  int num_subdivisions_per_laser_scan;
  int num_point_clouds;
  double rangefinder_sampling_ratio;
  double odometry_sampling_ratio;
  double imu_sampling_ratio;
};

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

// Try to convert 'msg' into 'options'. Returns false on failure.
bool FromRosMessage(const cartographer_ros_msgs::TrajectoryOptions& msg,
                    TrajectoryOptions* options);

// Converts 'trajectory_options' into a ROS message.
cartographer_ros_msgs::TrajectoryOptions ToRosMessage(
    const TrajectoryOptions& trajectory_options);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H_
