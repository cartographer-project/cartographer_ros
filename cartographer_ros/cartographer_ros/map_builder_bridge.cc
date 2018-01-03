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

#include "cartographer/common/make_unique.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer_ros/msg_conversion.h"

namespace cartographer_ros {

namespace {

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kConstraintMarkerScale = 0.025;

::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.ns = "Trajectory " + std::to_string(trajectory_id);
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.header.stamp = ::ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
  marker.scale.x = kTrajectoryLineStripMarkerScale;
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}

void PushAndResetLineMarker(visualization_msgs::Marker* marker,
                            std::vector<visualization_msgs::Marker>* markers) {
  if (marker->points.size() > 1) {
    markers->push_back(*marker);
    ++marker->id;
  }
  marker->points.clear();
}

}  // namespace

MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer) {}

void MapBuilderBridge::LoadMap(const std::string& map_filename) {
  LOG(INFO) << "Loading map '" << map_filename << "'...";
  cartographer::io::ProtoStreamReader stream(map_filename);
  map_builder_->LoadMap(&stream);
}

int MapBuilderBridge::AddTrajectory(
    const std::unordered_set<std::string>& expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      ::std::bind(&MapBuilderBridge::OnLocalSlamResult, this,
                  ::std::placeholders::_1, ::std::placeholders::_2,
                  ::std::placeholders::_3, ::std::placeholders::_4,
                  ::std::placeholders::_5));
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          trajectory_options.num_subdivisions_per_laser_scan,
          trajectory_options.tracking_frame,
          node_options_.lookup_transform_timeout_sec, tf_buffer_,
          map_builder_->GetTrajectoryBuilder(trajectory_id));
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 1);
  map_builder_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_->pose_graph()->RunFinalOptimization();
}

void MapBuilderBridge::SerializeState(const std::string& filename) {
  cartographer::io::ProtoStreamWriter writer(filename);
  map_builder_->SerializeState(&writer);
  CHECK(writer.Close()) << "Could not write state.";
}

bool MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  const std::string error =
      map_builder_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    return false;
  }

  CHECK(response_proto.textures_size() > 0)
      << "empty textures given for submap: " << submap_id;

  response.submap_version = response_proto.submap_version();
  for (const auto& texture_proto : response_proto.textures()) {
    response.textures.emplace_back();
    auto& texture = response.textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  return true;
}

cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  for (const auto& submap_id_data :
       map_builder_->pose_graph()->GetAllSubmapData()) {
    cartographer_ros_msgs::SubmapEntry submap_entry;
    submap_entry.trajectory_id = submap_id_data.id.trajectory_id;
    submap_entry.submap_index = submap_id_data.id.submap_index;
    submap_entry.submap_version = submap_id_data.data.submap->num_range_data();
    submap_entry.pose = ToGeometryMsgPose(submap_id_data.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}

std::unordered_map<int, MapBuilderBridge::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  std::unordered_map<int, TrajectoryState> trajectory_states;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    std::shared_ptr<const TrajectoryState::LocalSlamData> local_slam_data;
    {
      cartographer::common::MutexLocker lock(&mutex_);
      if (trajectory_state_data_.count(trajectory_id) == 0) {
        continue;
      }
      local_slam_data = trajectory_state_data_.at(trajectory_id);
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
    trajectory_states[trajectory_id] = {
        local_slam_data,
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(
            local_slam_data->time,
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};
  }
  return trajectory_states;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
  visualization_msgs::MarkerArray trajectory_node_list;
  const auto nodes = map_builder_->pose_graph()->GetTrajectoryNodes();
  for (const int trajectory_id : nodes.trajectory_ids()) {
    visualization_msgs::Marker marker =
        CreateTrajectoryMarker(trajectory_id, node_options_.map_frame);

    for (const auto& node_id_data : nodes.trajectory(trajectory_id)) {
      if (node_id_data.data.constant_data == nullptr) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        continue;
      }
      const ::geometry_msgs::Point node_point =
          ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
      marker.points.push_back(node_point);
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      if (marker.points.size() == 16384) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
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

  const auto trajectory_nodes =
      map_builder_->pose_graph()->GetTrajectoryNodes();
  const auto submap_data = map_builder_->pose_graph()->GetAllSubmapData();
  const auto constraints = map_builder_->pose_graph()->constraints();

  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::PoseGraph::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
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

    const auto submap_it = submap_data.find(constraint.submap_id);
    if (submap_it == submap_data.end()) {
      continue;
    }
    const auto& submap_pose = submap_it->data.pose;
    const auto node_it = trajectory_nodes.find(constraint.node_id);
    if (node_it == trajectory_nodes.end()) {
      continue;
    }
    const auto& trajectory_node_pose = node_it->data.global_pose;
    const cartographer::transform::Rigid3d constraint_pose =
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

void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id, const ::cartographer::common::Time time,
    const ::cartographer::transform::Rigid3d local_pose,
    ::cartographer::sensor::RangeData range_data_in_local,
    const std::unique_ptr<const ::cartographer::mapping::NodeId>) {
  std::shared_ptr<const TrajectoryState::LocalSlamData> local_slam_data =
      std::make_shared<TrajectoryState::LocalSlamData>(
          TrajectoryState::LocalSlamData{time, local_pose,
                                         std::move(range_data_in_local)});
  cartographer::common::MutexLocker lock(&mutex_);
  trajectory_state_data_[trajectory_id] = std::move(local_slam_data);
}

}  // namespace cartographer_ros
