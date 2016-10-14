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

#include "cartographer_ros/xray.h"

#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/xray_points_processor.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

void WriteXRayImages(const std::vector<::cartographer::mapping::TrajectoryNode>&
                         trajectory_nodes,
                     const double voxel_size, const std::string& stem) {
  carto::io::NullPointsProcessor null_points_processor;
  carto::io::XRayPointsProcessor xy_xray_points_processor(
      voxel_size, carto::transform::Rigid3f::Rotation(
                      Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitY())),
      stem + "_xray_xy.png", &null_points_processor);
  carto::io::XRayPointsProcessor yz_xray_points_processor(
      voxel_size, carto::transform::Rigid3f::Rotation(
                      Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())),
      stem + "_xray_yz.png", &xy_xray_points_processor);
  carto::io::XRayPointsProcessor xz_xray_points_processor(
      voxel_size, carto::transform::Rigid3f::Rotation(
                      Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitZ())),
      stem + "_xray_xz.png", &yz_xray_points_processor);

  for (const auto& node : trajectory_nodes) {
    const carto::sensor::LaserFan laser_fan = carto::sensor::TransformLaserFan(
        carto::sensor::Decompress(node.constant_data->laser_fan_3d),
        node.pose.cast<float>());

    carto::io::PointsBatch points_batch;
    points_batch.origin = laser_fan.origin;
    points_batch.points = laser_fan.returns;
    xz_xray_points_processor.Process(points_batch);
  }
  xz_xray_points_processor.Flush();
}

}  // namespace cartographer_ros
