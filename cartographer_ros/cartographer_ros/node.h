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
#include "tf2_ros/transform_listener.h"

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const NodeOptions& options);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void SpinForever();
  void Initialize();

 private:
  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);

  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void PublishPoseAndScanMatchedPointCloud(
      const ::ros::WallTimerEvent& timer_event);
  void SpinOccupancyGridThreadForever();

  const NodeOptions options_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  cartographer::common::Mutex mutex_;
  std::unique_ptr<MapBuilderBridge> map_builder_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Subscriber horizontal_laser_scan_subscriber_;
  std::vector<::ros::Subscriber> point_cloud_subscribers_;
  ::ros::Subscriber odometry_subscriber_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::ServiceServer submap_query_server_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;
  cartographer::common::Time last_scan_matched_point_cloud_time_ =
      cartographer::common::Time::min();
  ::ros::ServiceServer finish_trajectory_server_;

  ::ros::Publisher occupancy_grid_publisher_;
  std::thread occupancy_grid_thread_;
  bool terminating_ = false GUARDED_BY(mutex_);

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_NODE_H_
