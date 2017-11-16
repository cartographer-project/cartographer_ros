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

SubmapTexture::Pixels UnpackTextureData(const std::string& compressed_cells,
                                        const int width, const int height) {
  SubmapTexture::Pixels pixels;
  std::string cells;
  ::cartographer::common::FastGunzipString(compressed_cells, &cells);
  const int num_pixels = width * height;
  CHECK_EQ(cells.size(), 2 * num_pixels);
  pixels.intensity.reserve(num_pixels);
  pixels.alpha.reserve(num_pixels);
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      pixels.intensity.push_back(cells[(i * width + j) * 2]);
      pixels.alpha.push_back(cells[(i * width + j) * 2 + 1]);
    }
  }
  return pixels;
}

::cartographer::io::UniqueCairoSurfacePtr DrawTexture(
    const std::vector<char>& intensity, const std::vector<char>& alpha,
    const int width, const int height,
    std::vector<uint32_t>* const cairo_data) {
  CHECK(cairo_data->empty());

  // Properly dealing with a non-common stride would make this code much more
  // complicated. Let's check that it is not needed.
  const int expected_stride = 4 * width;
  CHECK_EQ(expected_stride, cairo_format_stride_for_width(
                                ::cartographer::io::kCairoFormat, width));
  for (size_t i = 0; i < intensity.size(); ++i) {
    // We use the red channel to track intensity information. The green
    // channel we use to track if a cell was ever observed.
    const uint8_t intensity_value = intensity.at(i);
    const uint8_t alpha_value = alpha.at(i);
    const uint8_t observed =
        (intensity_value == 0 && alpha_value == 0) ? 0 : 255;
    cairo_data->push_back((alpha_value << 24) | (intensity_value << 16) |
                          (observed << 8) | 0);
  }

  auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create_for_data(
          reinterpret_cast<unsigned char*>(cairo_data->data()),
          ::cartographer::io::kCairoFormat, width, height, expected_stride));
  CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS)
      << cairo_status_to_string(cairo_surface_status(surface.get()));
  return surface;
}

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
    const std::string compressed_cells(texture.cells.begin(),
                                       texture.cells.end());
    response->textures.emplace_back(SubmapTexture{
        UnpackTextureData(compressed_cells, texture.width, texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }
  return response;
}

}  // namespace cartographer_ros
