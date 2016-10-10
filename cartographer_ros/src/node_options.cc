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

#include "node_options.h"

#include "glog/logging.h"

namespace cartographer_ros {

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  NodeOptions options;
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());
  options.sensor_bridge_options = CreateSensorBridgeOptions(
      lua_parameter_dictionary->GetDictionary("sensor_bridge").get());
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.tracking_frame =
      lua_parameter_dictionary->GetString("tracking_frame");
  options.published_frame =
      lua_parameter_dictionary->GetString("published_frame");
  options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
  options.provide_odom_frame =
      lua_parameter_dictionary->GetBool("provide_odom_frame");
  options.use_odometry_data =
      lua_parameter_dictionary->GetBool("use_odometry_data");
  options.use_horizontal_laser =
      lua_parameter_dictionary->GetBool("use_horizontal_laser");
  options.use_horizontal_multi_echo_laser =
      lua_parameter_dictionary->GetBool("use_horizontal_multi_echo_laser");
  options.num_lasers_3d =
      lua_parameter_dictionary->GetNonNegativeInt("num_lasers_3d");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");

  CHECK_EQ(options.use_horizontal_laser +
               options.use_horizontal_multi_echo_laser +
               (options.num_lasers_3d > 0),
           1)
      << "Configuration error: 'use_horizontal_laser', "
         "'use_horizontal_multi_echo_laser' and 'num_lasers_3d' are "
         "mutually exclusive, but one is required.";
  CHECK_EQ(
      options.map_builder_options.use_trajectory_builder_2d(),
      options.use_horizontal_laser || options.use_horizontal_multi_echo_laser);
  CHECK_EQ(options.map_builder_options.use_trajectory_builder_3d(),
           options.num_lasers_3d > 0);
  return options;
}

}  // namespace cartographer_ros
