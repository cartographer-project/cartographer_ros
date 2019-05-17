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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "cartographer/common/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/time.hpp>

namespace cartographer_ros {

class MapBuilderBridge {
 public:
  struct TrajectoryState {
    // Contains the trajectory state data received from local SLAM, after
    // it had processed accumulated 'range_data_in_local' and estimated
    // current 'local_pose' at 'time'.
    struct LocalSlamData {
      ::cartographer::common::Time time;
      ::cartographer::transform::Rigid3d local_pose;
      ::cartographer::sensor::RangeData range_data_in_local;
    };
    std::shared_ptr<const LocalSlamData> local_slam_data;
    cartographer::transform::Rigid3d local_to_map;
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;
  };

  MapBuilderBridge(
      const NodeOptions& node_options,
      std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
      tf2_ros::Buffer* tf_buffer);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  void LoadState(const std::string& state_filename, bool load_frozen_state);
  int AddTrajectory(
      const std::set<
          ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& trajectory_options);
  void FinishTrajectory(int trajectory_id);
  void RunFinalOptimization();
  bool SerializeState(const std::string& filename);

  void HandleSubmapQuery(
      const std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Request> request,
      std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Response> response);

  std::set<int> GetFrozenTrajectoryIds();
  cartographer_ros_msgs::msg::SubmapList GetSubmapList(::rclcpp::Time node_time);
  std::unordered_map<int, TrajectoryState> GetTrajectoryStates()
      EXCLUDES(mutex_);
  visualization_msgs::msg::MarkerArray GetTrajectoryNodeList(::rclcpp::Time node_time);
  visualization_msgs::msg::MarkerArray GetLandmarkPosesList(::rclcpp::Time node_time);
  visualization_msgs::msg::MarkerArray GetConstraintList(::rclcpp::Time node_time);

  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  void OnLocalSlamResult(
      const int trajectory_id, const ::cartographer::common::Time time,
      const ::cartographer::transform::Rigid3d local_pose,
      ::cartographer::sensor::RangeData range_data_in_local,
      const std::unique_ptr<const ::cartographer::mapping::
                                TrajectoryBuilderInterface::InsertionResult>
          insertion_result) EXCLUDES(mutex_);

  cartographer::common::Mutex mutex_;
  const NodeOptions node_options_;
  std::unordered_map<int, std::shared_ptr<const TrajectoryState::LocalSlamData>>
      trajectory_state_data_ GUARDED_BY(mutex_);
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
  tf2_ros::Buffer* const tf_buffer_;

  std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
