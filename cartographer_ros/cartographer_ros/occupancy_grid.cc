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

#include "cartographer_ros/occupancy_grid.h"

#include "cartographer/common/port.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"

namespace {

Eigen::AlignedBox2f ComputeMapBoundingBox(
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes) {
  Eigen::AlignedBox2f bounding_box(Eigen::Vector2f::Zero());
  for (const auto& node : trajectory_nodes) {
    const auto& data = *node.constant_data;
    ::cartographer::sensor::RangeData range_data;
    if (!data.range_data_3d.returns.empty()) {
      range_data = ::cartographer::sensor::TransformRangeData(
          ::cartographer::sensor::Decompress(data.range_data_3d),
          node.pose.cast<float>());
    } else {
      range_data = ::cartographer::sensor::TransformRangeData(
          data.range_data_2d, node.pose.cast<float>());
    }
    bounding_box.extend(range_data.origin.head<2>());
    for (const Eigen::Vector3f& hit : range_data.returns) {
      bounding_box.extend(hit.head<2>());
    }
    for (const Eigen::Vector3f& miss : range_data.misses) {
      bounding_box.extend(miss.head<2>());
    }
  }
  return bounding_box;
}

}  // namespace

namespace cartographer_ros {

void BuildOccupancyGrid2D(
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes,
    const string& map_frame,
    const ::cartographer::mapping_2d::proto::SubmapsOptions& submaps_options,
    ::nav_msgs::OccupancyGrid* const occupancy_grid) {
  CHECK(!trajectory_nodes.empty());
  namespace carto = ::cartographer;
  const carto::mapping_2d::MapLimits map_limits =
      ComputeMapLimits(submaps_options.resolution(), trajectory_nodes);
  carto::mapping_2d::ProbabilityGrid probability_grid(map_limits);
  carto::mapping_2d::RangeDataInserter range_data_inserter(
      submaps_options.range_data_inserter_options());
  for (const auto& node : trajectory_nodes) {
    CHECK(node.constant_data->range_data_3d.returns.empty());  // No 3D yet.
    range_data_inserter.Insert(
        carto::sensor::TransformRangeData(node.constant_data->range_data_2d,
                                          node.pose.cast<float>()),
        &probability_grid);
  }

  // TODO(whess): Compute the latest time of in 'trajectory_nodes'.
  occupancy_grid->header.stamp = ToRos(trajectory_nodes.back().time());
  occupancy_grid->header.frame_id = map_frame;
  occupancy_grid->info.map_load_time = occupancy_grid->header.stamp;

  Eigen::Array2i offset;
  carto::mapping_2d::CellLimits cell_limits;
  probability_grid.ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = probability_grid.limits().resolution();

  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = cell_limits.num_y_cells;
  occupancy_grid->info.height = cell_limits.num_x_cells;

  occupancy_grid->info.origin.position.x =
      probability_grid.limits().max().x() -
      (offset.y() + cell_limits.num_y_cells) * resolution;
  occupancy_grid->info.origin.position.y =
      probability_grid.limits().max().y() -
      (offset.x() + cell_limits.num_x_cells) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;

  occupancy_grid->data.resize(cell_limits.num_x_cells * cell_limits.num_y_cells,
                              -1);
  for (const Eigen::Array2i& xy_index :
       carto::mapping_2d::XYIndexRangeIterator(cell_limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      const int value = carto::common::RoundToInt(
          (probability_grid.GetProbability(xy_index + offset) -
           carto::mapping::kMinProbability) *
          100. /
          (carto::mapping::kMaxProbability - carto::mapping::kMinProbability));
      CHECK_LE(0, value);
      CHECK_GE(100, value);
      occupancy_grid->data[(cell_limits.num_x_cells - xy_index.x()) *
                               cell_limits.num_y_cells -
                           xy_index.y() - 1] = value;
    }
  }
}

::cartographer::mapping_2d::MapLimits ComputeMapLimits(
    const double resolution,
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes) {
  Eigen::AlignedBox2f bounding_box = ComputeMapBoundingBox(trajectory_nodes);
  // Add some padding to ensure all rays are still contained in the map after
  // discretization.
  const float kPadding = 3.f * resolution;
  bounding_box.min() -= kPadding * Eigen::Vector2f::Ones();
  bounding_box.max() += kPadding * Eigen::Vector2f::Ones();
  const Eigen::Vector2d pixel_sizes =
      bounding_box.sizes().cast<double>() / resolution;
  return ::cartographer::mapping_2d::MapLimits(
      resolution, bounding_box.max().cast<double>(),
      ::cartographer::mapping_2d::CellLimits(
          ::cartographer::common::RoundToInt(pixel_sizes.y()),
          ::cartographer::common::RoundToInt(pixel_sizes.x())));
}

}  // namespace cartographer_ros
