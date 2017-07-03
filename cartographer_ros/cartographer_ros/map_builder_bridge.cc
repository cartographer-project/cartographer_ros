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

#include "cartographer_ros/map_builder_bridge.h"

#include "cartographer_ros/assets_writer.h"
#include "cartographer_ros/color.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/occupancy_grid.h"

namespace cartographer_ros {

constexpr double kTrajectoryLineStripMarkerScale = 0.07;

MapBuilderBridge::MapBuilderBridge(const NodeOptions& node_options,
                                   tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_(node_options.map_builder_options),
      tf_buffer_(tf_buffer) {}

int MapBuilderBridge::AddTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  const int trajectory_id = map_builder_.AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options);
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          trajectory_options.tracking_frame,
          node_options_.lookup_transform_timeout_sec, tf_buffer_,
          map_builder_.GetTrajectoryBuilder(trajectory_id));
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 1);
  map_builder_.FinishTrajectory(trajectory_id);
  map_builder_.sparse_pose_graph()->RunFinalOptimization();
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::SerializeState(const std::string& stem) {
  std::ofstream proto_file(stem + ".pb",
                           std::ios_base::out | std::ios_base::binary);
  CHECK(map_builder_.sparse_pose_graph()->ToProto().SerializeToOstream(
      &proto_file))
      << "Could not serialize pose graph.";
  proto_file.close();
  CHECK(proto_file) << "Could not write pose graph.";
}

void MapBuilderBridge::WriteAssets(const string& stem) {
  const auto all_trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  if (!HasNonTrimmedNode(all_trajectory_nodes)) {
    LOG(WARNING) << "No data was collected and no assets will be written.";
    return;
  }
  // Make sure there is a trajectory with id = 0.
  CHECK_EQ(trajectory_options_.count(0), 1);
  LOG(INFO) << "Writing assets with stem '" << stem << "'...";
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    // We arbitrarily use the submap_options() from the first trajectory to
    // write the 2D assets.
    Write2DAssets(
        all_trajectory_nodes, node_options_.map_frame,
        trajectory_options_[0]
            .trajectory_builder_options.trajectory_builder_2d_options()
            .submaps_options(),
        stem);
  }

  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    Write3DAssets(
        all_trajectory_nodes,
        trajectory_options_[0]
            .trajectory_builder_options.trajectory_builder_3d_options()
            .submaps_options()
            .high_resolution(),
        stem);
  }
}

bool MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  const std::string error = map_builder_.SubmapToProto(
      cartographer::mapping::SubmapId{request.trajectory_id,
                                      request.submap_index},
      &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    return false;
  }

  response.submap_version = response_proto.submap_version();
  response.cells.insert(response.cells.begin(), response_proto.cells().begin(),
                        response_proto.cells().end());
  response.width = response_proto.width();
  response.height = response_proto.height();
  response.resolution = response_proto.resolution();
  response.slice_pose = ToGeometryMsgPose(
      cartographer::transform::ToRigid3(response_proto.slice_pose()));
  return true;
}

cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  const auto all_submap_data =
      map_builder_.sparse_pose_graph()->GetAllSubmapData();
  for (size_t trajectory_id = 0; trajectory_id < all_submap_data.size();
       ++trajectory_id) {
    for (size_t submap_index = 0;
         submap_index < all_submap_data[trajectory_id].size(); ++submap_index) {
      const auto& submap_data = all_submap_data[trajectory_id][submap_index];
      if (submap_data.submap == nullptr) {
        continue;
      }
      cartographer_ros_msgs::SubmapEntry submap_entry;
      submap_entry.trajectory_id = trajectory_id;
      submap_entry.submap_index = submap_index;
      submap_entry.submap_version = submap_data.submap->num_range_data();
      submap_entry.pose = ToGeometryMsgPose(submap_data.pose);
      submap_list.submap.push_back(submap_entry);
    }
  }
  return submap_list;
}

std::unique_ptr<nav_msgs::OccupancyGrid>
MapBuilderBridge::BuildOccupancyGrid() {
  CHECK(node_options_.map_builder_options.use_trajectory_builder_2d())
      << "Publishing OccupancyGrids for 3D data is not yet supported";
  const auto all_trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  std::unique_ptr<nav_msgs::OccupancyGrid> occupancy_grid;
  if (HasNonTrimmedNode(all_trajectory_nodes)) {
    occupancy_grid =
        cartographer::common::make_unique<nav_msgs::OccupancyGrid>();
    // Make sure there is a trajectory with id = 0.
    CHECK_EQ(trajectory_options_.count(0), 1);
    BuildOccupancyGrid2D(
        all_trajectory_nodes, node_options_.map_frame,
        trajectory_options_[0]
            .trajectory_builder_options.trajectory_builder_2d_options()
            .submaps_options(),
        occupancy_grid.get());
  }
  return occupancy_grid;
}

std::unordered_map<int, MapBuilderBridge::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  std::unordered_map<int, TrajectoryState> trajectory_states;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    const cartographer::mapping::TrajectoryBuilder* const trajectory_builder =
        map_builder_.GetTrajectoryBuilder(trajectory_id);
    const cartographer::mapping::TrajectoryBuilder::PoseEstimate pose_estimate =
        trajectory_builder->pose_estimate();
    if (cartographer::common::ToUniversal(pose_estimate.time) < 0) {
      continue;
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
    trajectory_states[trajectory_id] = {
        pose_estimate,
        map_builder_.sparse_pose_graph()->GetLocalToGlobalTransform(
            trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(
            pose_estimate.time,
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};
  }
  return trajectory_states;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
  visualization_msgs::MarkerArray trajectory_node_list;
  const auto all_trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  int marker_id = 0;
  for (int trajectory_id = 0;
       trajectory_id < static_cast<int>(all_trajectory_nodes.size());
       ++trajectory_id) {
    const auto& single_trajectory_nodes = all_trajectory_nodes[trajectory_id];
    visualization_msgs::Marker marker;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.stamp = ::ros::Time::now();
    marker.header.frame_id = node_options_.map_frame;
    marker.color = GetColor(trajectory_id);
    marker.scale.x = kTrajectoryLineStripMarkerScale;
    marker.pose.orientation.w = 1.0;
    for (const auto& node : single_trajectory_nodes) {
      if (node.trimmed()) {
        continue;
      }
      const ::geometry_msgs::Point node_point = ToGeometryMsgPoint(
          (node.pose * node.constant_data->tracking_to_pose).translation());
      marker.points.push_back(node_point);
      // Work around the 16384 point limit in rviz by splitting the
      // trajectory into multiple markers.
      if (marker.points.size() == 16384) {
        trajectory_node_list.markers.push_back(marker);
        marker.id = marker_id++;
        marker.points.clear();
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    trajectory_node_list.markers.push_back(marker);
  }
  return trajectory_node_list;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

}  // namespace cartographer_ros
