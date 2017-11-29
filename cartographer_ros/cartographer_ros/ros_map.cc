/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer_ros/ros_map.h"

#include "cartographer/common/make_unique.h"

namespace cartographer_ros {

std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const std::string& frame_id, const ros::Time& time, const double resolution,
    const Eigen::Array2f& origin, cairo_surface_t* surface) {
  auto occupancy_grid =
      ::cartographer::common::make_unique<nav_msgs::OccupancyGrid>();

  const int width = cairo_image_surface_get_width(surface);
  const int height = cairo_image_surface_get_height(surface);
  const ros::Time now = ros::Time::now();

  occupancy_grid->header.stamp = time;
  occupancy_grid->header.frame_id = frame_id;
  occupancy_grid->info.map_load_time = time;
  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = width;
  occupancy_grid->info.height = height;
  occupancy_grid->info.origin.position.x = -origin.x() * resolution;
  occupancy_grid->info.origin.position.y = (-height + origin.y()) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;

  const uint32_t* pixel_data =
      reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(surface));
  occupancy_grid->data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value = observed == 0 ? -1 : ::cartographer::common::RoundToInt(
                                                 (1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid->data.push_back(value);
    }
  }

  return occupancy_grid;
}

void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer) {
  const std::string header = "P5\n# Cartographer map; " +
                             std::to_string(resolution) + " m/pixel\n" +
                             std::to_string(image.width()) + " " +
                             std::to_string(image.height()) + "\n255\n";
  file_writer->Write(header.data(), header.size());
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const char color = image.GetPixel(x, y)[0];
      file_writer->Write(&color, 1);
    }
  }
}

void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename,
               ::cartographer::io::FileWriter* file_writer) {
  // Magic constants taken directly from ros map_saver code:
  // https://github.com/ros-planning/navigation/blob/ac41d2480c4cf1602daf39a6e9629142731d92b0/map_server/src/map_saver.cpp#L114
  const std::string output =
      "image: " + pgm_filename + "\n" + "resolution: " +
      std::to_string(resolution) + "\n" + "origin: [" +
      std::to_string(origin.x()) + ", " + std::to_string(origin.y()) +
      ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  file_writer->Write(output.data(), output.size());
}

}  // namespace cartographer_ros
