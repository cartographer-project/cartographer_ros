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

#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/SubmapQuery.h"

namespace cartographer_ros {

std::unique_ptr<SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    ros::ServiceClient* client) {
  ::cartographer_ros_msgs::SubmapQuery srv;
  srv.request.trajectory_id = submap_id.trajectory_id;
  srv.request.submap_index = submap_id.submap_index;
  if (!client->call(srv)) {
    return nullptr;
  }
  CHECK(!srv.response.textures.empty());
  auto response = ::cartographer::common::make_unique<SubmapTextures>();
  response->version = srv.response.submap_version;
  for (const auto& texture : srv.response.textures) {
    std::string compressed_cells(texture.cells.begin(), texture.cells.end());
    std::string cells;
    ::cartographer::common::FastGunzipString(compressed_cells, &cells);
    const int num_pixels = texture.width * texture.height;
    CHECK_EQ(cells.size(), 2 * num_pixels);
    std::vector<char> intensity;
    intensity.reserve(num_pixels);
    std::vector<char> alpha;
    alpha.reserve(num_pixels);
    for (int i = 0; i < texture.height; ++i) {
      for (int j = 0; j < texture.width; ++j) {
        intensity.push_back(cells[(i * texture.width + j) * 2]);
        alpha.push_back(cells[(i * texture.width + j) * 2 + 1]);
      }
    }
    response->textures.emplace_back(
        SubmapTexture{intensity, alpha, texture.width, texture.height,
                      texture.resolution, ToRigid3d(texture.slice_pose)});
  }
  return response;
}

}  // namespace cartographer_ros
