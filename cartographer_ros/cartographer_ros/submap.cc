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

#include "cartographer_ros/submap.h"

#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/SubmapQuery.h"

namespace cartographer_ros {

std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    ros::ServiceClient* client) {
  ::cartographer_ros_msgs::SubmapQuery srv;
  srv.request.trajectory_id = submap_id.trajectory_id;
  srv.request.submap_index = submap_id.submap_index;
  if (!client->call(srv) ||
      srv.response.status.code != ::cartographer_ros_msgs::StatusCode::OK) {
    return nullptr;
  }
  if (srv.response.textures.empty()) {
    return nullptr;
  }
  auto response = absl::make_unique<::cartographer::io::SubmapTextures>();
  response->version = srv.response.submap_version;
  for (const auto& texture : srv.response.textures) {
    const std::string compressed_cells(texture.cells.begin(),
                                       texture.cells.end());
    response->textures.emplace_back(::cartographer::io::SubmapTexture{
        ::cartographer::io::UnpackTextureData(compressed_cells, texture.width,
                                              texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }
  return response;
}

bool Has2DGrid(const ::cartographer::mapping::proto::Submap& submap) {
  return submap.has_submap_2d() && submap.submap_2d().has_grid();
}

bool Has3DGrids(const ::cartographer::mapping::proto::Submap& submap) {
  return submap.has_submap_3d() &&
         submap.submap_3d().has_low_resolution_hybrid_grid() &&
         submap.submap_3d().has_high_resolution_hybrid_grid();
}

}  // namespace cartographer_ros
