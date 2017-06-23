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

#ifndef CARTOGRAPHER_ROS_ASSETS_WRITER_H_
#define CARTOGRAPHER_ROS_ASSETS_WRITER_H_

#include <string>
#include <vector>

#include "cartographer/mapping/trajectory_node.h"
#include "cartographer_ros/node_options.h"

namespace cartographer_ros {

// Returns 'true' if there is at least one untrimmed node for any trajectory.
// The Write?DAssets functions expects this to be 'true'.
bool HasNonTrimmedNode(
    const std::vector<std::vector<::cartographer::mapping::TrajectoryNode>>&
        all_trajectory_nodes);

// Writes a trajectory proto and an occupancy grid.
void Write2DAssets(
    const std::vector<std::vector<::cartographer::mapping::TrajectoryNode>>&
        all_trajectory_nodes,
    const string& map_frame,
    const ::cartographer::mapping_2d::proto::SubmapsOptions& submaps_options,
    const std::string& stem);

// Writes X-ray images, trajectory proto, and PLY files from the
// 'all_trajectory_nodes'. The filenames will all start with 'stem'.
void Write3DAssets(
    const std::vector<std::vector<::cartographer::mapping::TrajectoryNode>>&
        all_trajectory_nodes,
    const double voxel_size, const std::string& stem);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_ASSETS_WRITER_H_
