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

#ifndef CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_
#define CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_

#include <memory>
#include <unordered_set>

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "nav_msgs/OccupancyGrid.h"

namespace cartographer_ros {

class MapBuilderBridge {
 public:
  MapBuilderBridge(const NodeOptions& options,
                   const std::unordered_set<string>& expected_sensor_ids,
                   tf2_ros::Buffer* tf_buffer);
  ~MapBuilderBridge();

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);

  cartographer_ros_msgs::SubmapList GetSubmapList();
  std::unique_ptr<nav_msgs::OccupancyGrid> BuildOccupancyGrid();

  SensorBridge* sensor_bridge() { return sensor_bridge_.get(); }

  // TODO(damonkohler): Remove these accessors.
  int trajectory_id() const { return trajectory_id_; }
  TfBridge* tf_bridge() { return &tf_bridge_; }
  cartographer::mapping::MapBuilder* map_builder() { return &map_builder_; }

 private:
  const NodeOptions options_;

  std::deque<cartographer::mapping::TrajectoryNode::ConstantData>
      constant_data_;
  cartographer::mapping::MapBuilder map_builder_;
  std::unordered_set<string> expected_sensor_ids_;
  int trajectory_id_ = -1;
  TfBridge tf_bridge_;
  std::unique_ptr<SensorBridge> sensor_bridge_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H_
