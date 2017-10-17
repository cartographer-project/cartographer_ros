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

#ifndef CARTOGRAPHER_ROS_SUBMAP_H_
#define CARTOGRAPHER_ROS_SUBMAP_H_

#include <memory>
#include <vector>

#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "ros/ros.h"

namespace cartographer_ros {

struct SubmapTexture {
  std::vector<char> intensity;
  std::vector<char> alpha;
  int width;
  int height;
  double resolution;
  ::cartographer::transform::Rigid3d slice_pose;
};

struct SubmapTextures {
  int version;
  std::vector<SubmapTexture> textures;
};

// Fetch 'submap_id' using the 'client' and returning the response or 'nullptr'
// on error.
std::unique_ptr<SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    ros::ServiceClient* client);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_SUBMAP_H_
