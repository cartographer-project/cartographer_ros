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

#include "occupancy_grid.h"

#include "time_conversion.h"

#include "glog/logging.h"

namespace cartographer_ros {

void BuildOccupancyGrid(
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes,
    const NodeOptions& options,
    ::nav_msgs::OccupancyGrid* const occupancy_grid) {
  namespace carto = ::cartographer;
  CHECK(options.map_builder_options.use_trajectory_builder_2d())
      << "Publishing OccupancyGrids for 3D data is not yet supported";
  const auto& submaps_options =
      options.map_builder_options.trajectory_builder_2d_options()
          .submaps_options();
  const carto::mapping_2d::MapLimits map_limits =
      carto::mapping_2d::MapLimits::ComputeMapLimits(
          submaps_options.resolution(), trajectory_nodes);
  carto::mapping_2d::ProbabilityGrid probability_grid(map_limits);
  carto::mapping_2d::LaserFanInserter laser_fan_inserter(
      submaps_options.laser_fan_inserter_options());
  for (const auto& node : trajectory_nodes) {
    CHECK(node.constant_data->laser_fan_3d.returns.empty());  // No 3D yet.
    laser_fan_inserter.Insert(
        carto::sensor::TransformLaserFan(
            node.constant_data->laser_fan,
            carto::transform::Project2D(node.pose).cast<float>()),
        &probability_grid);
  }

  occupancy_grid->header.stamp = ToRos(trajectory_nodes.back().time());
  occupancy_grid->header.frame_id = options.map_frame;
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

}  // namespace cartographer_ros
