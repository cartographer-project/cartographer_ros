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
#include <string>
#include <vector>

#include "cartographer/io/image.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "ros/ros.h"

namespace cartographer_ros {

struct SubmapTexture {
  struct Pixels {
    std::vector<char> intensity;
    std::vector<char> alpha;
  };
  Pixels pixels;
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

// Unpacks cell data as provided by the backend into 'intensity' and 'alpha'.
SubmapTexture::Pixels UnpackTextureData(const std::string& compressed_cells,
                                        int width, int height);

// Draw a texture into a cairo surface. 'cairo_data' will store the pixel data
// for the surface and must therefore outlive the use of the surface.
::cartographer::io::UniqueCairoSurfacePtr DrawTexture(
    const std::vector<char>& intensity, const std::vector<char>& alpha,
    int width, int height, std::vector<uint32_t>* cairo_data);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_SUBMAP_H_
