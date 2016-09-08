-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  expect_odometry_data = false,
  publish_occupancy_grid = false,
  expect_horizontal_laser = false,
  expect_horizontal_multi_echo_laser = true,
  horizontal_laser_min_range = 0.,
  horizontal_laser_max_range = 30.,
  horizontal_laser_missing_echo_ray_length = 5.,
  num_lasers_3d = 2,
  lookup_transform_timeout_sec = 0.01,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}

options.map_builder.use_trajectory_builder_3d = true
options.sparse_pose_graph.optimize_every_n_scans = 320
options.sparse_pose_graph.constraint_builder.sampling_ratio = 0.03
options.sparse_pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- Reuse the coarser 3D voxel filter to speed up the computation of loop closure
-- constraints.
options.sparse_pose_graph.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
options.sparse_pose_graph.constraint_builder.min_score = 0.62
options.sparse_pose_graph.constraint_builder.log_matches = true

options.trajectory_builder.scans_per_accumulation = 20

return options
