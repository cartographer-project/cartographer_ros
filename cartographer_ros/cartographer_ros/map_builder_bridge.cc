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

#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer_ros/assets_writer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/occupancy_grid.h"
#include "cartographer_ros_msgs/TrajectorySubmapList.h"

namespace cartographer_ros {

MapBuilderBridge::MapBuilderBridge(const NodeOptions& options,
                                   tf2_ros::Buffer* const tf_buffer)
    : options_(options),
      map_builder_(options.map_builder_options),
      tf_buffer_(tf_buffer) {}

int MapBuilderBridge::AddTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const string& tracking_frame) {
  const int trajectory_id = map_builder_.AddTrajectoryBuilder(
      expected_sensor_ids, options_.trajectory_builder_options);
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
  std::vector<::cartographer::mapping::TrajectoryNode> trajectory_nodes;
  for (const auto& single_trajectory :
       map_builder_.sparse_pose_graph()->GetTrajectoryNodes()) {
    trajectory_nodes.insert(trajectory_nodes.end(), single_trajectory.begin(),
                            single_trajectory.end());
  }
  if (trajectory_nodes.empty()) {
    LOG(WARNING) << "No data was collected and no assets will be written.";
  } else {
    LOG(INFO) << "Writing assets with stem '" << stem << "'...";
    if (options_.map_builder_options.use_trajectory_builder_2d()) {
      Write2DAssets(
          trajectory_nodes, options_.map_frame,
          options_.trajectory_builder_options.trajectory_builder_2d_options()
              .submaps_options(),
          stem);
    }

    if (options_.map_builder_options.use_trajectory_builder_3d()) {
      Write3DAssets(trajectory_nodes, options_.trajectory_builder_options
                                          .trajectory_builder_3d_options()
                                          .submaps_options()
                                          .high_resolution(),
                    stem);
    }
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
    const std::vector<cartographer::transform::Rigid3d> submap_transforms =
        map_builder_.sparse_pose_graph()->GetSubmapTransforms(trajectory_id);
    const cartographer::mapping::Submaps* submaps =
        map_builder_.GetTrajectoryBuilder(trajectory_id)->submaps();
    CHECK_LE(submap_transforms.size(), submaps->size());

    cartographer_ros_msgs::TrajectorySubmapList trajectory_submap_list;
    for (size_t submap_index = 0; submap_index != submap_transforms.size();
         ++submap_index) {
      cartographer_ros_msgs::SubmapEntry submap_entry;
      submap_entry.submap_version = submaps->Get(submap_index)->num_range_data;
      submap_entry.pose = ToGeometryMsgPose(submap_transforms[submap_index]);
      trajectory_submap_list.submap.push_back(submap_entry);
    }
    submap_list.trajectory.push_back(trajectory_submap_list);
  }
  return submap_list;
}

std::unique_ptr<nav_msgs::OccupancyGrid>
MapBuilderBridge::BuildOccupancyGrid() {
  CHECK(options_.map_builder_options.use_trajectory_builder_2d())
      << "Publishing OccupancyGrids for 3D data is not yet supported";
  std::vector<::cartographer::mapping::TrajectoryNode> trajectory_nodes;
  for (const auto& single_trajectory :
       map_builder_.sparse_pose_graph()->GetTrajectoryNodes()) {
    trajectory_nodes.insert(trajectory_nodes.end(), single_trajectory.begin(),
                            single_trajectory.end());
  }
  std::unique_ptr<nav_msgs::OccupancyGrid> occupancy_grid;
  if (!trajectory_nodes.empty()) {
    occupancy_grid =
        cartographer::common::make_unique<nav_msgs::OccupancyGrid>();
    BuildOccupancyGrid2D(
        trajectory_nodes, options_.map_frame,
        options_.trajectory_builder_options.trajectory_builder_2d_options()
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

    trajectory_states[trajectory_id] = {
        pose_estimate,
        map_builder_.sparse_pose_graph()->GetLocalToGlobalTransform(
            trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(pose_estimate.time,
                                                   options_.published_frame)};
  }
  return trajectory_states;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodesList() {
  visualization_msgs::MarkerArray trajectory_nodes_list;

  for (int trajectory_id = 0;
       trajectory_id < map_builder_.num_trajectory_builders();
       ++trajectory_id) {
    const auto trajectory_nodes =
        map_builder_.sparse_pose_graph()->GetTrajectoryNodes();

    int i = 0;
    for (const auto& single_trajectory : trajectory_nodes) {
      for (const auto& node : single_trajectory) {
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
  }
  return trajectory_nodes_list;
}

std::unordered_map<std::string, visualization_msgs::MarkerArray>
MapBuilderBridge::GetConstraintsList() {
  std::unordered_map<std::string, visualization_msgs::MarkerArray>
      constraints_viz;

  const auto trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  const auto constraints = map_builder_.sparse_pose_graph()->constraints();
  std::vector<cartographer::transform::Rigid3d> submap_transforms;
  for (auto& single_trajectory : trajectory_nodes) {
    for (auto& node : single_trajectory) {
    auto current_trajectory_transforms =
        map_builder_.sparse_pose_graph()->GetSubmapTransforms(
            node.constant_data->trajectory_id);
    std::move(current_trajectory_transforms.begin(),
              current_trajectory_transforms.end(),
              std::back_inserter(submap_transforms));
    }
  }

  int id_inter = 0;
  std::string ns_inter = "inter";
  int id_intra = 0;
  std::string ns_intra = "intra";
  ros::Time now = ros::Time::now();
  for (const auto& constraint : constraints) {
    visualization_msgs::Marker constraint_marker, residual_error_marker;
    int* id_cnt;
    std::string* tag;
    std_msgs::ColorRGBA color_constraint, color_error;

    if (constraint.tag ==
        cartographer::mapping::SparsePoseGraph::Constraint::INTER_SUBMAP) {
      id_cnt = &id_inter;
      tag = &ns_inter;

      // yellow
      color_constraint.a = 1.0;
      color_constraint.r = color_constraint.g = 1.0;

      // cyan
      color_error.a = 1.0;
      color_error.b = color_error.g = 1.0;
    } else {
      id_cnt = &id_intra;
      tag = &ns_intra;
      color_constraint.a = color_error.a = 1.0;
      color_constraint.g = 1.0;
      color_error.b = 1.0;
    }

    constraint_marker = createVisualizationMarker(
        (*id_cnt)++, visualization_msgs::Marker::LINE_STRIP, "constraint", now,
        options_.map_frame);
    constraint_marker.color = color_constraint;
    residual_error_marker = createVisualizationMarker(
        (*id_cnt)++, visualization_msgs::Marker::LINE_STRIP, "residual error",
        now, options_.map_frame);
    residual_error_marker.color = color_error;

    geometry_msgs::Point submap_point, submap_pose_point, trajectory_node_point;

    submap_point = ToGeometryMsgPoint(
        submap_transforms[constraint.submap_id.submap_index]);
    submap_pose_point = ToGeometryMsgPoint(
        submap_transforms[constraint.submap_id.submap_index] *
        constraint.pose.zbar_ij);
    trajectory_node_point = ToGeometryMsgPoint(
        trajectory_nodes[constraint.node_id.trajectory_id][constraint.node_id.node_index].pose);

    constraint_marker.points.push_back(submap_point);
    constraint_marker.points.push_back(submap_pose_point);
    residual_error_marker.points.push_back(submap_pose_point);
    residual_error_marker.points.push_back(trajectory_node_point);
    constraints_viz[*tag].markers.push_back(constraint_marker);
    constraints_viz[*tag].markers.push_back(residual_error_marker);
  }
  return constraints_viz;
}

visualization_msgs::Marker createVisualizationMarker(
    const int id, const int type, const std::string ns, const ros::Time time,
    const std::string frame_id) {
  visualization_msgs::Marker m;
  m.id = id;
  m.type = type;
  m.ns = ns;
  m.header.stamp = time;
  m.header.frame_id = frame_id;
  m.color.a = 1.0;
  m.scale.x = 0.02;
  return m;
}

}  // namespace cartographer_ros
