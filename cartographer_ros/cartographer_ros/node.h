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

#ifndef CARTOGRAPHER_ROS_NODE_H_
#define CARTOGRAPHER_ROS_NODE_H_

#include <memory>
#include <vector>

#include "cartographer/common/mutex.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/SensorTopics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "cartographer_ros_msgs/WriteAssets.h"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const NodeOptions& node_options, tf2_ros::Buffer* tf_buffer);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  ::ros::NodeHandle* node_handle();
  MapBuilderBridge* map_builder_bridge();

 private:
  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);
  bool HandleWriteAssets(
      cartographer_ros_msgs::WriteAssets::Request& request,
      cartographer_ros_msgs::WriteAssets::Response& response);
  int AddTrajectory(const TrajectoryOptions& options,
                    const cartographer_ros_msgs::SensorTopics& topics);
  void LaunchSubscribers(const TrajectoryOptions& options,
                         const cartographer_ros_msgs::SensorTopics& topics,
                         int trajectory_id);
  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void SpinOccupancyGridThreadForever();
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicName(const ::cartographer_ros_msgs::SensorTopics& topics,
                         const TrajectoryOptions& options);

  const NodeOptions node_options_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  cartographer::common::Mutex mutex_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<::ros::ServiceServer> service_servers_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;
  cartographer::common::Time last_scan_matched_point_cloud_time_ =
      cartographer::common::Time::min();

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, ::ros::Subscriber> laser_scan_subscribers_;
  std::unordered_map<int, ::ros::Subscriber> multi_echo_laser_scan_subscribers_;
  std::unordered_map<int, ::ros::Subscriber> odom_subscribers_;
  std::unordered_map<int, ::ros::Subscriber> imu_subscribers_;
  std::unordered_map<int, std::vector<::ros::Subscriber>>
      point_cloud_subscribers_;
  std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_;
  std::thread occupancy_grid_thread_;
  bool terminating_ = false GUARDED_BY(mutex_);

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_NODE_H_
