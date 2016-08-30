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

include "trajectory_builder.lua"
include "sparse_pose_graph.lua"

options = {
  sparse_pose_graph = SPARSE_POSE_GRAPH,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  odom_frame = "odom",
  tracking_frame = "base_link",
  provide_odom_frame = false,
  expect_odometry_data = false,
  publish_occupancy_grid = false,
  laser_min_range = 0.,
  laser_max_range = 30.,
  laser_missing_echo_ray_length = 5.,
  lookup_transform_timeout = 0.01,
  use_laser_scan_2d = true
}

options.trajectory_builder.expect_imu_data = false
options.trajectory_builder.use_online_correlative_scan_matching = true

return options
