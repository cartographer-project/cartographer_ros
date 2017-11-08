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

#include "cartographer_ros/trajectory_options.h"

#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"

namespace cartographer_ros {

namespace {

void CheckTrajectoryOptions(const TrajectoryOptions& options) {
  CHECK_GE(options.num_subdivisions_per_laser_scan, 1);
  CHECK_GE(options.num_laser_scans + options.num_multi_echo_laser_scans +
               options.num_point_clouds,
           1)
      << "Configuration error: 'num_laser_scans', "
         "'num_multi_echo_laser_scans' and 'num_point_clouds' are "
         "all zero, but at least one is required.";
}

}  // namespace

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
  options.num_laser_scans =
      lua_parameter_dictionary->GetNonNegativeInt("num_laser_scans");
  options.num_multi_echo_laser_scans =
      lua_parameter_dictionary->GetNonNegativeInt("num_multi_echo_laser_scans");
  options.num_subdivisions_per_laser_scan =
      lua_parameter_dictionary->GetNonNegativeInt(
          "num_subdivisions_per_laser_scan");
  options.num_point_clouds =
      lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");
  options.rangefinder_sampling_ratio =
      lua_parameter_dictionary->GetDouble("rangefinder_sampling_ratio");
  options.odometry_sampling_ratio =
      lua_parameter_dictionary->GetDouble("odometry_sampling_ratio");
  options.imu_sampling_ratio =
      lua_parameter_dictionary->GetDouble("imu_sampling_ratio");
  if (lua_parameter_dictionary->HasKey("initial_trajectory_pose"))
  {
    auto initial_pose_dictionnary =
      lua_parameter_dictionary->GetDictionary("initial_trajectory_pose").get();
    options.initial_trajectory_pose.to_trajectory_id =
      initial_pose_dictionnary->GetInt("to_trajectory_id");
    options.initial_trajectory_pose.relative_pose =
      cartographer::transform::FromDictionary(initial_pose_dictionnary);
  }
  else
  {
    options.initial_trajectory_pose.to_trajectory_id = -1;
    options.initial_trajectory_pose.relative_pose = ::cartographer::transform::Rigid3d::Identity();
  }
  options.initial_trajectory_pose.time = ::cartographer::common::Time::min();
  CheckTrajectoryOptions(options);
  return options;
}

bool FromRosMessage(const cartographer_ros_msgs::TrajectoryOptions& msg,
                    TrajectoryOptions* options) {
  options->tracking_frame = msg.tracking_frame;
  options->published_frame = msg.published_frame;
  options->odom_frame = msg.odom_frame;
  options->provide_odom_frame = msg.provide_odom_frame;
  options->use_odometry = msg.use_odometry;
  options->num_laser_scans = msg.num_laser_scans;
  options->num_multi_echo_laser_scans = msg.num_multi_echo_laser_scans;
  options->num_subdivisions_per_laser_scan =
      msg.num_subdivisions_per_laser_scan;
  options->num_point_clouds = msg.num_point_clouds;
  options->rangefinder_sampling_ratio = msg.rangefinder_sampling_ratio;
  options->odometry_sampling_ratio = msg.odometry_sampling_ratio;
  options->imu_sampling_ratio = msg.imu_sampling_ratio;
  options->initial_trajectory_pose.to_trajectory_id =
      msg.initial_trajectory_pose.to_trajectory_id;
  options->initial_trajectory_pose.relative_pose =
    ToRigid3d(msg.initial_trajectory_pose.relative_pose);
  options->initial_trajectory_pose.time =
    FromRos(msg.initial_trajectory_pose.stamp);
  if (!options->trajectory_builder_options.ParseFromString(
          msg.trajectory_builder_options_proto)) {
    LOG(ERROR) << "Failed to parse protobuf";
    return false;
  }
  CheckTrajectoryOptions(*options);
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
  msg.num_laser_scans = options.num_laser_scans;
  msg.num_multi_echo_laser_scans = options.num_multi_echo_laser_scans;
  msg.num_subdivisions_per_laser_scan = options.num_subdivisions_per_laser_scan;
  msg.num_point_clouds = options.num_point_clouds;
  msg.rangefinder_sampling_ratio = options.rangefinder_sampling_ratio;
  msg.odometry_sampling_ratio = options.odometry_sampling_ratio;
  msg.imu_sampling_ratio = options.imu_sampling_ratio;
  msg.initial_trajectory_pose.to_trajectory_id =
    options.initial_trajectory_pose.to_trajectory_id;
  msg.initial_trajectory_pose.relative_pose =
    ToGeometryMsgPose(options.initial_trajectory_pose.relative_pose);
  msg.initial_trajectory_pose.stamp =
    ToRos(options.initial_trajectory_pose.time);
  options.trajectory_builder_options.SerializeToString(
      &msg.trajectory_builder_options_proto);
  return msg;
}

}  // namespace cartographer_ros
