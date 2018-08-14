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

#include <string>
#include <vector>

#include "absl/memory/memory.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "gflags/gflags.h"
#include "ros/ros.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

DEFINE_string(initial_pose, "", "Starting pose of a new trajectory");

namespace cartographer_ros {
namespace {

TrajectoryOptions LoadOptions() {
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{FLAGS_configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  auto lua_parameter_dictionary =
      cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          code, std::move(file_resolver));
  if (!FLAGS_initial_pose.empty()) {
    auto initial_trajectory_pose_file_resolver =
        absl::make_unique<cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{FLAGS_configuration_directory});
    auto initial_trajectory_pose =
        cartographer::common::LuaParameterDictionary::NonReferenceCounted(
            "return " + FLAGS_initial_pose,
            std::move(initial_trajectory_pose_file_resolver));
    return CreateTrajectoryOptions(lua_parameter_dictionary.get(),
                                   initial_trajectory_pose.get());
  } else {
    return CreateTrajectoryOptions(lua_parameter_dictionary.get());
  }
}

bool Run() {
  ros::NodeHandle node_handle;
  ros::ServiceClient client =
      node_handle.serviceClient<cartographer_ros_msgs::StartTrajectory>(
          kStartTrajectoryServiceName);
  cartographer_ros_msgs::StartTrajectory srv;
  srv.request.options = ToRosMessage(LoadOptions());
  srv.request.topics.laser_scan_topic = node_handle.resolveName(
      kLaserScanTopic, true /* apply topic remapping */);
  srv.request.topics.multi_echo_laser_scan_topic =
      node_handle.resolveName(kMultiEchoLaserScanTopic, true);
  srv.request.topics.point_cloud2_topic =
      node_handle.resolveName(kPointCloud2Topic, true);
  srv.request.topics.imu_topic = node_handle.resolveName(kImuTopic, true);
  srv.request.topics.odometry_topic =
      node_handle.resolveName(kOdometryTopic, true);

  if (!client.call(srv)) {
    LOG(ERROR) << "Failed to call " << kStartTrajectoryServiceName << ".";
    return false;
  }
  if (srv.response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Error starting trajectory - message: '"
               << srv.response.status.message
               << "' (status code: " << std::to_string(srv.response.status.code)
               << ").";
    return false;
  }
  LOG(INFO) << "Started trajectory " << srv.response.trajectory_id;
  return true;
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\n\n"
      "Convenience tool around the start_trajectory service. This takes a Lua "
      "file that is accepted by the node as well and starts a new trajectory "
      "using its settings.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_start_trajectory");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  int exit_code = cartographer_ros::Run() ? 0 : 1;
  ::ros::shutdown();
  return exit_code;
}
