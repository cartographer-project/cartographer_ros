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

#include "cartographer_ros/assets_writer.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/ply_writing_points_processor.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/xray_points_processor.h"
#include "cartographer/mapping_2d/proto/laser_fan_inserter_options.pb.h"
#include "cartographer/proto/trajectory.pb.h"
#include "cartographer_ros/map_writer.h"
#include "cartographer_ros/occupancy_grid.h"
#include "nav_msgs/OccupancyGrid.h"

namespace cartographer_ros {

namespace {

namespace carto = ::cartographer;

// Writes an occupany grid.
void Write2DAssets(const std::vector<::cartographer::mapping::TrajectoryNode>&
                       trajectory_nodes,
                   const NodeOptions& options, const std::string& stem) {
  ::nav_msgs::OccupancyGrid occupancy_grid;
  BuildOccupancyGrid(trajectory_nodes, options, &occupancy_grid);
  WriteOccupancyGridToPgmAndYaml(occupancy_grid, stem);
}

// Writes X-ray images and PLY files from the 'trajectory_nodes'. The filenames
// will all start with 'stem'.
void Write3DAssets(const std::vector<::cartographer::mapping::TrajectoryNode>&
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
  carto::io::PlyWritingPointsProcessor ply_writing_points_processor(
      stem + ".ply", &xz_xray_points_processor);

  for (const auto& node : trajectory_nodes) {
    const carto::sensor::LaserFan laser_fan = carto::sensor::TransformLaserFan(
        carto::sensor::Decompress(node.constant_data->laser_fan_3d),
        node.pose.cast<float>());

    auto points_batch = carto::common::make_unique<carto::io::PointsBatch>();
    points_batch->origin = laser_fan.origin;
    points_batch->points = laser_fan.returns;
    for (const uint8 reflectivity :
         node.constant_data->laser_fan_3d.reflectivities) {
      points_batch->colors.push_back(
          carto::io::Color{{reflectivity, reflectivity, reflectivity}});
    }
    ply_writing_points_processor.Process(std::move(points_batch));
  }
  ply_writing_points_processor.Flush();
}

}  // namespace

void WriteAssets(const std::vector<::cartographer::mapping::TrajectoryNode>&
                     trajectory_nodes,
                 const NodeOptions& options, const std::string& stem) {
  // Write the trajectory.
  std::ofstream proto_file(stem + ".pb",
                           std::ios_base::out | std::ios_base::binary);
  const carto::proto::Trajectory trajectory =
      carto::mapping::ToProto(trajectory_nodes);
  CHECK(trajectory.SerializeToOstream(&proto_file))
      << "Could not serialize trajectory.";
  proto_file.close();
  CHECK(proto_file) << "Could not write trajectory.";

  if (options.map_builder_options.use_trajectory_builder_2d()) {
    Write2DAssets(trajectory_nodes, options, stem);
  }

  if (options.map_builder_options.use_trajectory_builder_3d()) {
    Write3DAssets(trajectory_nodes,
                  options.map_builder_options.trajectory_builder_3d_options()
                      .submaps_options()
                      .high_resolution(),
                  stem);
  }
}

}  // namespace cartographer_ros
