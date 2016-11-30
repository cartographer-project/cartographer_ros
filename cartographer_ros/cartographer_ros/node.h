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
#include "cartographer_ros/node_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectorySubmapList.h"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kSubmapQueryServiceName[] = "submap_query";

// Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const NodeOptions& options, tf2_ros::Buffer* tf_buffer);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void Initialize();

  ::ros::NodeHandle* node_handle();
  MapBuilderBridge* map_builder_bridge();

 private:
  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);

  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event);
  void SpinOccupancyGridThreadForever();

  const NodeOptions options_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  cartographer::common::Mutex mutex_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);
  int trajectory_id_ = -1;
  std::unordered_set<string> expected_sensor_ids_;

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::ServiceServer submap_query_server_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;
  cartographer::common::Time last_scan_matched_point_cloud_time_ =
      cartographer::common::Time::min();

  ::ros::Publisher occupancy_grid_publisher_;
  std::thread occupancy_grid_thread_;
  bool terminating_ = false GUARDED_BY(mutex_);

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_NODE_H_
