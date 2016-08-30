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

include "trajectory_builder_3d.lua"
include "sparse_pose_graph.lua"

options = {
  sparse_pose_graph = SPARSE_POSE_GRAPH,
  trajectory_builder = TRAJECTORY_BUILDER_3D,
  map_frame = "map",
  odom_frame = "odom",
  tracking_frame = "base_link",
  provide_odom_frame = true,
  expect_odometry_data = false,
  publish_occupancy_grid = false,
  laser_min_range = 0.,
  laser_max_range = 30.,
  laser_missing_echo_ray_length = 5.,
  lookup_transform_timeout = 0.01,
  num_lasers_3d = 2
}

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
