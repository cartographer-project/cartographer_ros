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

#include "cartographer_ros/pcd.h"

#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/pcd_points_processor.h"

#include <iostream>
#include <fstream>


namespace cartographer_ros {

namespace carto = ::cartographer;

void WritePCDFile(const std::vector<::cartographer::mapping::TrajectoryNode>&
                         trajectory_nodes,
                     const double voxel_size, const std::string& stem) {
  carto::io::NullPointsProcessor null_points_processor;
  carto::io::PCDPointsProcessor pc_point_processor(
      voxel_size,
      stem + "_pointcloud.pcd", &null_points_processor);

  for (const auto& node : trajectory_nodes) {
    const carto::sensor::LaserFan laser_fan = carto::sensor::TransformLaserFan(
        carto::sensor::Decompress(node.constant_data->laser_fan_3d),
        node.pose.cast<float>());

    carto::io::PointsBatch points_batch;
    points_batch.origin = laser_fan.origin;
    points_batch.points = laser_fan.returns;
    pc_point_processor.Process(points_batch);
  }
  pc_point_processor.Flush();
}

}  // namespace cartographer_ros
