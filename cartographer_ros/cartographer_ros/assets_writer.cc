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
#include "cartographer/common/time.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/ply_writing_points_processor.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/xray_points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping_2d/proto/range_data_inserter_options.pb.h"
#include "cartographer_ros/map_writer.h"
#include "cartographer_ros/occupancy_grid.h"
#include "nav_msgs/OccupancyGrid.h"

namespace cartographer_ros {

namespace {

namespace carto = ::cartographer;

void WriteTrajectory(const std::vector<::cartographer::mapping::TrajectoryNode>&
                         trajectory_nodes,
                     const std::string& stem) {
  carto::mapping::proto::Trajectory trajectory;
  // TODO(whess): Add multi-trajectory support.
  for (const auto& node : trajectory_nodes) {
    const auto& data = *node.constant_data;
    auto* node_proto = trajectory.add_node();
    node_proto->set_timestamp(carto::common::ToUniversal(data.time));
    *node_proto->mutable_pose() =
        carto::transform::ToProto(node.pose * data.tracking_to_pose);
  }

  // Write the trajectory.
  std::ofstream proto_file(stem + ".pb",
                           std::ios_base::out | std::ios_base::binary);
  CHECK(trajectory.SerializeToOstream(&proto_file))
      << "Could not serialize trajectory.";
  proto_file.close();
  CHECK(proto_file) << "Could not write trajectory.";
}

}  // namespace

// Writes an occupancy grid.
void Write2DAssets(
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes,
    const string& map_frame,
    const ::cartographer::mapping_2d::proto::SubmapsOptions& submaps_options,
    const std::string& stem) {
  WriteTrajectory(trajectory_nodes, stem);

  ::nav_msgs::OccupancyGrid occupancy_grid;
  BuildOccupancyGrid2D(trajectory_nodes, map_frame, submaps_options,
                       &occupancy_grid);
  WriteOccupancyGridToPgmAndYaml(occupancy_grid, stem);
}

// Writes X-ray images and PLY files from the 'trajectory_nodes'. The filenames
// will all start with 'stem'.
void Write3DAssets(const std::vector<::cartographer::mapping::TrajectoryNode>&
                       trajectory_nodes,
                   const double voxel_size, const std::string& stem) {
  WriteTrajectory(trajectory_nodes, stem);

  const auto file_writer_factory = [](const string& filename) {
    return carto::common::make_unique<carto::io::StreamFileWriter>(filename);
  };

  carto::io::NullPointsProcessor null_points_processor;
  carto::io::XRayPointsProcessor xy_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitY())),
      {}, stem + "_xray_xy", file_writer_factory, &null_points_processor);
  carto::io::XRayPointsProcessor yz_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())),
      {}, stem + "_xray_yz", file_writer_factory, &xy_xray_points_processor);
  carto::io::XRayPointsProcessor xz_xray_points_processor(
      voxel_size,
      carto::transform::Rigid3f::Rotation(
          Eigen::AngleAxisf(-M_PI / 2.f, Eigen::Vector3f::UnitZ())),
      {}, stem + "_xray_xz", file_writer_factory, &yz_xray_points_processor);
  carto::io::PlyWritingPointsProcessor ply_writing_points_processor(
      file_writer_factory(stem + ".ply"), &xz_xray_points_processor);

  for (const auto& node : trajectory_nodes) {
    const carto::sensor::RangeData range_data =
        carto::sensor::TransformRangeData(
            carto::sensor::Decompress(node.constant_data->range_data_3d),
            node.pose.cast<float>());

    auto points_batch = carto::common::make_unique<carto::io::PointsBatch>();
    points_batch->origin = range_data.origin;
    points_batch->points = range_data.returns;
    ply_writing_points_processor.Process(std::move(points_batch));
  }
  ply_writing_points_processor.Flush();
}

}  // namespace cartographer_ros
