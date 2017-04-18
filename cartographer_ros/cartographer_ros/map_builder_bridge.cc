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
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/occupancy_grid.h"
#include "cartographer_ros_msgs/TrajectorySubmapList.h"

namespace cartographer_ros {

MapBuilderBridge::MapBuilderBridge(const NodeOptions& options,
                                   tf2_ros::Buffer* const tf_buffer)
    : options_(options),
      map_builder_(options.map_builder_options, &constant_data_),
      tf_buffer_(tf_buffer) {}

int MapBuilderBridge::AddTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const string& tracking_frame) {
  const int trajectory_id =
      map_builder_.AddTrajectoryBuilder(expected_sensor_ids);
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          tracking_frame, options_.lookup_transform_timeout_sec, tf_buffer_,
          map_builder_.GetTrajectoryBuilder(trajectory_id));
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  CHECK_EQ(sensor_bridges_.count(trajectory_id), 1);
  map_builder_.FinishTrajectory(trajectory_id);
  map_builder_.sparse_pose_graph()->RunFinalOptimization();
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::WriteAssets(const string& stem) {
  const auto trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  if (trajectory_nodes.empty()) {
    LOG(WARNING) << "No data was collected and no assets will be written.";
  } else {
    LOG(INFO) << "Writing assets with stem '" << stem << "'...";
    cartographer_ros::WriteAssets(trajectory_nodes, options_, stem);
  }
}

bool MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  const std::string error = map_builder_.SubmapToProto(
      request.trajectory_id, request.submap_index, &response_proto);
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
  submap_list.header.frame_id = options_.map_frame;
  for (int trajectory_id = 0;
       trajectory_id < map_builder_.num_trajectory_builders();
       ++trajectory_id) {
    const cartographer::mapping::Submaps* submaps =
        map_builder_.GetTrajectoryBuilder(trajectory_id)->submaps();
    const std::vector<cartographer::transform::Rigid3d> submap_transforms =
        map_builder_.sparse_pose_graph()->GetSubmapTransforms(*submaps);
    CHECK_EQ(submap_transforms.size(), submaps->size());

    cartographer_ros_msgs::TrajectorySubmapList trajectory_submap_list;
    for (int submap_index = 0; submap_index != submaps->size();
         ++submap_index) {
      cartographer_ros_msgs::SubmapEntry submap_entry;
      submap_entry.submap_version =
          submaps->Get(submap_index)->end_range_data_index;
      submap_entry.pose = ToGeometryMsgPose(submap_transforms[submap_index]);
      submap_entry.local_pose = ToGeometryMsgPose(submaps->Get(submap_index)->local_pose());
      trajectory_submap_list.submap.push_back(submap_entry);
    }
    submap_list.trajectory.push_back(trajectory_submap_list);
  }
  return submap_list;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodesList() {
  visualization_msgs::MarkerArray trajectory_nodes_list;

  for (int trajectory_id = 0;
      trajectory_id < map_builder_.num_trajectory_builders();
      ++trajectory_id) {
    const auto trajectory_nodes = map_builder_.sparse_pose_graph()->GetTrajectoryNodes();

    int i = 0;
    for (const auto& node : trajectory_nodes){
      visualization_msgs::Marker marker;
      marker.id = i++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.header.stamp = ::ros::Time::now();
      marker.header.frame_id = options_.map_frame;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.pose = ToGeometryMsgPose(node.pose);
      trajectory_nodes_list.markers.push_back(marker);
    }
  }

  return trajectory_nodes_list;
}

cartographer_ros_msgs::ConstraintVisualization MapBuilderBridge::GetConstraintsList() {
  cartographer_ros_msgs::ConstraintVisualization constraint_visualization;

  const std::vector<cartographer::transform::Rigid3d> submap_transforms =
      map_builder_.sparse_pose_graph()->GetSubmapTransforms();

  const auto trajectory_nodes = map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  const auto constraints = map_builder_.sparse_pose_graph()->constraints();

  int i = 0;
  for (const auto &constraint : constraints) {

    // add constraint only if INTRA_SUBMAP

    visualization_msgs::Marker constraint_marker, residual_error_marker;

    // creating constraint line strip marker (green)
    constraint_marker.id = i++;
    constraint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    constraint_marker.header.stamp = ::ros::Time::now();
    constraint_marker.header.frame_id = options_.map_frame;
    constraint_marker.color.g = 1.0;
    constraint_marker.color.a = 1.0;
    constraint_marker.scale.x = 0.02;

    // creating residual error line strip marker (blue)
    residual_error_marker.id = i++;
    residual_error_marker.type = visualization_msgs::Marker::LINE_STRIP;
    residual_error_marker.header.stamp = ::ros::Time::now();
    residual_error_marker.header.frame_id = options_.map_frame;
    residual_error_marker.color.b = 1.0;
    residual_error_marker.color.a = 1.0;
    residual_error_marker.scale.x = 0.02;

    // creating points for the line strip markers
    geometry_msgs::Point submap_point, submap_pose_point, trajectory_node_point;

    submap_point.x = submap_transforms[constraint.i].translation().x();
    submap_point.y = submap_transforms[constraint.i].translation().y();
    submap_point.z = submap_transforms[constraint.i].translation().z();

    trajectory_node_point.x = trajectory_nodes[constraint.j].pose.translation().x();
    trajectory_node_point.y = trajectory_nodes[constraint.j].pose.translation().y();
    trajectory_node_point.z = trajectory_nodes[constraint.j].pose.translation().z();

    cartographer::transform::Rigid3d residual = submap_transforms[constraint.i] * constraint.pose.zbar_ij;
    submap_pose_point.x = residual.translation().x();
    submap_pose_point.y = residual.translation().y();
    submap_pose_point.z = residual.translation().z();

    constraint_marker.points.push_back(submap_point);
    constraint_marker.points.push_back(submap_pose_point);
    constraint_visualization.constraints.markers.push_back(constraint_marker);

    residual_error_marker.points.push_back(submap_pose_point);
    residual_error_marker.points.push_back(trajectory_node_point);
    constraint_visualization.residual_errors.markers.push_back(residual_error_marker);
  }
  return constraint_visualization;
}

std::unique_ptr<nav_msgs::OccupancyGrid>
MapBuilderBridge::BuildOccupancyGrid() {
  const auto trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  std::unique_ptr<nav_msgs::OccupancyGrid> occupancy_grid;
  if (!trajectory_nodes.empty()) {
    occupancy_grid =
        cartographer::common::make_unique<nav_msgs::OccupancyGrid>();
    cartographer_ros::BuildOccupancyGrid(trajectory_nodes, options_,
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

    trajectory_states[trajectory_id] = {
        pose_estimate,
        map_builder_.sparse_pose_graph()->GetLocalToGlobalTransform(
            *trajectory_builder->submaps()),
        sensor_bridge.tf_bridge().LookupToTracking(pose_estimate.time,
                                                   options_.published_frame)};
  }
  return trajectory_states;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

}  // namespace cartographer_ros
