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

#include "map_writer.h"

#include <fstream>

#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

namespace cartographer_ros {

namespace {

void WriteOccupancyGridToPgm(const ::nav_msgs::OccupancyGrid& grid,
                             const std::string& filename) {
  LOG(INFO) << "Saving map to '" << filename << "'...";
  std::ofstream pgm_file(filename, std::ios::out | std::ios::binary);
  const std::string header = "P5\n# Cartographer map; " +
                             std::to_string(grid.info.resolution) +
                             " m/pixel\n" + std::to_string(grid.info.width) +
                             " " + std::to_string(grid.info.height) + "\n255\n";
  pgm_file.write(header.data(), header.size());
  for (size_t y = 0; y < grid.info.height; ++y) {
    for (size_t x = 0; x < grid.info.width; ++x) {
      const size_t i = x + (grid.info.height - y - 1) * grid.info.width;
      if (grid.data[i] >= 0 && grid.data[i] <= 100) {
        pgm_file.put((100 - grid.data[i]) * 255 / 100);
      } else {
        // We choose a value between the free and occupied threshold.
        constexpr uint8_t kUnknownValue = 128;
        pgm_file.put(kUnknownValue);
      }
    }
  }
  pgm_file.close();
  CHECK(pgm_file) << "Writing '" << filename << "' failed.";
}

void WriteOccupancyGridInfoToYaml(const ::nav_msgs::OccupancyGrid& grid,
                                  const std::string& map_filename,
                                  const std::string& yaml_filename) {
  LOG(INFO) << "Saving map info to '" << yaml_filename << "'...";
  std::ofstream yaml_file(yaml_filename, std::ios::out | std::ios::binary);
  {
    YAML::Emitter out(yaml_file);
    out << YAML::BeginMap;
    // TODO(whess): Use basename only?
    out << YAML::Key << "image" << YAML::Value << map_filename;
    out << YAML::Key << "resolution" << YAML::Value << grid.info.resolution;
    // According to map_server documentation "many parts of the system currently
    // ignore yaw" so it is good we use a zero value.
    constexpr double kYawButMaybeIgnored = 0.;
    out << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << grid.info.origin.position.x << grid.info.origin.position.y
        << kYawButMaybeIgnored << YAML::EndSeq;
    out << YAML::Key << "occupied_thresh" << YAML::Value << 0.51;
    out << YAML::Key << "free_thresh" << YAML::Value << 0.49;
    out << YAML::Key << "negate" << YAML::Value << 0;
    out << YAML::EndMap;
    CHECK(out.good()) << out.GetLastError();
  }
  yaml_file.close();
  CHECK(yaml_file) << "Writing '" << yaml_filename << "' failed.";
}

}  // namespace

void WriteOccupancyGridToPgmAndYaml(
    const ::nav_msgs::OccupancyGrid& occupancy_grid, const std::string& stem) {
  const std::string pgm_filename = stem + ".pgm";
  WriteOccupancyGridToPgm(occupancy_grid, pgm_filename);
  WriteOccupancyGridInfoToYaml(occupancy_grid, pgm_filename, stem + ".yaml");
}

}  // namespace cartographer_ros
