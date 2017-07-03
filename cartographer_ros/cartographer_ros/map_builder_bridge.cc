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

#include "cartographer/io/proto_stream.h"
#include "cartographer_ros/assets_writer.h"
#include "cartographer_ros/color.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/occupancy_grid.h"

namespace cartographer_ros {

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kConstraintMarkerScale = 0.025;

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
  cartographer::io::ProtoStreamWriter writer(stem + ".pbstream");
  map_builder_.SerializeState(&writer);
  CHECK(writer.Close()) << "Could not write state.";
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
    marker.pose.position.z = 0.05;
    for (const auto& node : single_trajectory_nodes) {
      if (node.trimmed()) {
        continue;
      }
      // In 2D, the pose in node.pose is xy-aligned. Multiplying by
      // node.constant_data->tracking_to_pose would give the full orientation,
      // but that is not needed here since we are only interested in the
      // translational part.
      const ::geometry_msgs::Point node_point =
          ToGeometryMsgPoint(node.pose.translation());
      marker.points.push_back(node_point);
      // Work around the 16384 point limit in RViz by splitting the
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

visualization_msgs::MarkerArray MapBuilderBridge::GetConstraintList() {
  visualization_msgs::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = ros::Time::now();
  constraint_intra_marker.header.frame_id = node_options_.map_frame;
  constraint_intra_marker.scale.x = kConstraintMarkerScale;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  residual_intra_marker.pose.position.z = 0.1;

  visualization_msgs::Marker constraint_inter_marker = constraint_intra_marker;
  constraint_inter_marker.id = marker_id++;
  constraint_inter_marker.ns = "Inter constraints";
  constraint_inter_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_marker = constraint_intra_marker;
  residual_inter_marker.id = marker_id++;
  residual_inter_marker.ns = "Inter residuals";
  residual_inter_marker.pose.position.z = 0.1;

  const auto all_trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  const auto all_submap_data =
      map_builder_.sparse_pose_graph()->GetAllSubmapData();
  const auto constraints = map_builder_.sparse_pose_graph()->constraints();

  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::SparsePoseGraph::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      color_constraint = GetColor(constraint.submap_id.submap_index +
                                  constraint.submap_id.trajectory_id + 25);
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      constraint_marker = &constraint_inter_marker;
      residual_marker = &residual_inter_marker;
      // Bright yellow
      color_constraint.a = 1.0;
      color_constraint.r = color_constraint.g = 1.0;
      // Bright cyan
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    const auto& submap_data =
        all_submap_data[constraint.submap_id.trajectory_id]
                       [constraint.submap_id.submap_index];
    const auto& submap_pose = submap_data.pose;
    // Similar to GetTrajectoryNodeList(), we can skip multiplying with
    // node.constant_data->tracking_to_pose.
    const auto& trajectory_node_pose =
        all_trajectory_nodes[constraint.node_id.trajectory_id]
                            [constraint.node_id.node_index]
                                .pose;
    // Similar to GetTrajectoryNodeList(), we can skip multiplying with
    // node.constant_data->tracking_to_pose.
    const ::cartographer::transform::Rigid3d constraint_pose =
        submap_pose * constraint.pose.zbar_ij;

    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_marker);
  constraint_list.markers.push_back(residual_inter_marker);
  return constraint_list;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

}  // namespace cartographer_ros
