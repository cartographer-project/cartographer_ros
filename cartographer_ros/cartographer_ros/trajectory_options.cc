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

#include "cartographer_ros/node_options.h"

#include "glog/logging.h"

namespace cartographer_ros {

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  TrajectoryOptions options;
  options.trajectory_builder_options =
      ::cartographer::mapping::CreateTrajectoryBuilderOptions(
          lua_parameter_dictionary->GetDictionary("trajectory_builder").get());
  options.tracking_frame =
      lua_parameter_dictionary->GetString("tracking_frame");
  options.published_frame =
      lua_parameter_dictionary->GetString("published_frame");
  options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
  options.provide_odom_frame =
      lua_parameter_dictionary->GetBool("provide_odom_frame");
  options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
  options.use_laser_scan = lua_parameter_dictionary->GetBool("use_laser_scan");
  options.use_multi_echo_laser_scan =
      lua_parameter_dictionary->GetBool("use_multi_echo_laser_scan");
  options.num_point_clouds =
      lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");

  CHECK_EQ(options.use_laser_scan + options.use_multi_echo_laser_scan +
               (options.num_point_clouds > 0),
           1)
      << "Configuration error: 'use_laser_scan', "
         "'use_multi_echo_laser_scan' and 'num_point_clouds' are "
         "mutually exclusive, but one is required.";

  return options;
}

bool FromRosMessage(const cartographer_ros_msgs::TrajectoryOptions& msg,
                    TrajectoryOptions* options) {
  options->tracking_frame = msg.tracking_frame;
  options->published_frame = msg.published_frame;
  options->odom_frame = msg.odom_frame;
  options->provide_odom_frame = msg.provide_odom_frame;
  options->use_odometry = msg.use_odometry;
  options->use_laser_scan = msg.use_laser_scan;
  options->use_multi_echo_laser_scan = msg.use_multi_echo_laser_scan;
  options->num_point_clouds = msg.num_point_clouds;
  if (!options->trajectory_builder_options.ParseFromString(
          msg.trajectory_builder_options_proto)) {
    LOG(ERROR) << "Failed to parse protobuf";
    return false;
  }
  return true;
}

cartographer_ros_msgs::TrajectoryOptions ToRosMessage(
    const TrajectoryOptions& options) {
  cartographer_ros_msgs::TrajectoryOptions msg;
  msg.tracking_frame = options.tracking_frame;
  msg.published_frame = options.published_frame;
  msg.odom_frame = options.odom_frame;
  msg.provide_odom_frame = options.provide_odom_frame;
  msg.use_odometry = options.use_odometry;
  msg.use_laser_scan = options.use_laser_scan;
  msg.use_multi_echo_laser_scan = options.use_multi_echo_laser_scan;
  msg.num_point_clouds = options.num_point_clouds;
  options.trajectory_builder_options.SerializeToString(
      &msg.trajectory_builder_options_proto);
  return msg;
}

}  // namespace cartographer_ros
