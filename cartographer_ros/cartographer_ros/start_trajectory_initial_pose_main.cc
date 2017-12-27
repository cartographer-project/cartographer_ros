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

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include <tf/tf.h>

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

boost::shared_ptr<ros::NodeHandle> nh;
ros::Subscriber initialPoseSub;

TrajectoryOptions LoadOptions(const geometry_msgs::PoseWithCovarianceStamped pose_msg) {
  std::string initial_pose=
      "{to_trajectory_id = 0, relative_pose = { translation = { "
      + std::to_string(pose_msg.pose.pose.position.x)
      +", "
      +std::to_string(pose_msg.pose.pose.position.y)
      +", "
      +"0. }, rotation = { 0., 0., "
      +std::to_string(tf::getYaw(pose_msg.pose.pose.orientation))
      +", } } }"
      ;
  ROS_INFO(initial_pose.c_str());
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<std::string>{FLAGS_configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  auto lua_parameter_dictionary =
      cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          code, std::move(file_resolver));

  auto initial_trajectory_pose_file_resolver =
      cartographer::common::make_unique<
          cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{FLAGS_configuration_directory});
  auto initial_trajectory_pose =
      cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          "return " + initial_pose,
          std::move(initial_trajectory_pose_file_resolver));
  return CreateTrajectoryOptions(lua_parameter_dictionary.get(),
                                 initial_trajectory_pose.get());
}



void HandleInitialPose(const geometry_msgs::PoseWithCovarianceStamped pose_msg){
  if(pose_msg.header.frame_id=="map"){
    ROS_INFO("Setting pose");
    ros::ServiceClient client_finish =
        nh->serviceClient<cartographer_ros_msgs::FinishTrajectory>(
            kFinishTrajectoryServiceName);
    cartographer_ros_msgs::FinishTrajectory srv_finish;
    int id_to_finish=1;
    srv_finish.request.trajectory_id=id_to_finish;
    bool trajectoryMaxIdReached=false;
    int trajectoryMaxID=100;

    while (!client_finish.call(srv_finish) && !trajectoryMaxIdReached) {
      LOG(ERROR) << "Error finishing trajectory, trying next ID.";
      id_to_finish++;
      srv_finish.request.trajectory_id=id_to_finish;
      trajectoryMaxIdReached=(id_to_finish>=trajectoryMaxID);
    }
    if (!trajectoryMaxIdReached){
      LOG(INFO) << "Finished trajectory " << srv_finish.request.trajectory_id;

      ros::ServiceClient client_start =
          nh->serviceClient<cartographer_ros_msgs::StartTrajectory>(
              kStartTrajectoryServiceName);
      cartographer_ros_msgs::StartTrajectory srv_start;
      srv_start.request.options = ToRosMessage(LoadOptions(pose_msg));
      srv_start.request.topics.laser_scan_topic = nh->resolveName(
          kLaserScanTopic, true /* apply topic remapping */);
      srv_start.request.topics.multi_echo_laser_scan_topic =
          nh->resolveName(kMultiEchoLaserScanTopic, true);
      srv_start.request.topics.point_cloud2_topic =
          nh->resolveName(kPointCloud2Topic, true);
      srv_start.request.topics.imu_topic = nh->resolveName(kImuTopic, true);
      srv_start.request.topics.odometry_topic =
          nh->resolveName(kOdometryTopic, true);

      if (!client_start.call(srv_start)) {
        LOG(ERROR) << "Error starting trajectory.";
      }
      LOG(INFO) << "Started trajectory " << srv_start.response.trajectory_id;
    }else{
      ROS_ERROR("Could not finish previous trajectory");
    }
  }else{
    ROS_ERROR("Pose needs to be specified in map frame");
  }
}

void Run() {
  initialPoseSub=nh->subscribe("/initialpose",1, &cartographer_ros::HandleInitialPose);
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

  ::ros::init(argc, argv, "cartographer_start_trajectory_initial_pose");
  cartographer_ros::nh = boost::make_shared<ros::NodeHandle> ("");

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ros::spin();
  return 0;
}
