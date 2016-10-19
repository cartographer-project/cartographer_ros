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

namespace cartographer_ros {

// Writes X-ray images and PLY files from the 'trajectory_nodes'. The filenames
// will all start with 'stem'.
void WriteAssets(const std::vector<::cartographer::mapping::TrajectoryNode>&
                     trajectory_nodes,
                 double voxel_size, const std::string& stem);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_ASSETS_WRITER_H_
