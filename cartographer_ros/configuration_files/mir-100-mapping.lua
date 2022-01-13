-- Copyright 2018 The Cartographer Authors
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
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_frame",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_pose_extrapolator = true,
  use_nav_sat = false,
  use_landmarks = true,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER.collate_landmarks = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- more points
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400
-- slightly slower insertion
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.53
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.493
-- slightly shorter rays
TRAJECTORY_BUILDER_2D.max_range = 15.
-- wheel odometry is fine
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
-- IMU is ok
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20

-- less outliers
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.
POSE_GRAPH.constraint_builder.min_score = 0.5
-- tune down IMU in optimization
POSE_GRAPH.optimization_problem.acceleration_weight = 0.1 * 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 0.1 * 3e5
-- ignore wheels in optimization
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.
POSE_GRAPH.optimization_problem.log_solver_summary = true

return options

