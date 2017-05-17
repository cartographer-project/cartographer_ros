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

#include "cartographer/common/time.h"
#include "cartographer/sensor/range_data.h"
#include "gtest/gtest.h"
#include "ros/ros.h"

namespace cartographer_ros {
namespace {

TEST(OccupancyGridTest, ComputeMapLimits) {
  constexpr int kTrajectoryId = 0;
  const ::cartographer::mapping::TrajectoryNode::ConstantData constant_data{
      ::cartographer::common::FromUniversal(52),
      ::cartographer::sensor::RangeData{
          Eigen::Vector3f::Zero(),
          {Eigen::Vector3f(-30.f, 1.f, 0.f), Eigen::Vector3f(50.f, -10.f, 0.f)},
          {}},
      ::cartographer::sensor::Compress(
          ::cartographer::sensor::RangeData{Eigen::Vector3f::Zero(), {}, {}}),
      kTrajectoryId, ::cartographer::transform::Rigid3d::Identity()};
  const ::cartographer::mapping::TrajectoryNode trajectory_node{
      &constant_data, ::cartographer::transform::Rigid3d::Identity()};
  constexpr double kResolution = 0.05;
  const ::cartographer::mapping_2d::MapLimits limits =
      ComputeMapLimits(kResolution, {trajectory_node});
  constexpr float kPaddingAwareTolerance = 5 * kResolution;
  EXPECT_NEAR(50.f, limits.max().x(), kPaddingAwareTolerance);
  EXPECT_NEAR(1.f, limits.max().y(), kPaddingAwareTolerance);
  EXPECT_LT(200, limits.cell_limits().num_x_cells);
  EXPECT_LT(1600, limits.cell_limits().num_y_cells);
  EXPECT_GT(400, limits.cell_limits().num_x_cells);
  EXPECT_GT(2000, limits.cell_limits().num_y_cells);
}

}  // namespace
}  // namespace cartographer_ros
