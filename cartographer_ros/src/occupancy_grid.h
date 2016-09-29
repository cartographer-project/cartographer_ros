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

#ifndef CARTOGRAHPER_ROS_OCCUPANCY_GRID_H_
#define CARTOGRAHPER_ROS_OCCUPANCY_GRID_H_

#include <vector>

#include "cartographer/mapping/trajectory_node.h"
#include "nav_msgs/OccupancyGrid.h"
#include "node_options.h"

namespace cartographer_ros {

void BuildOccupancyGrid(
    const std::vector<::cartographer::mapping::TrajectoryNode>&
        trajectory_nodes,
    const NodeOptions& options, ::nav_msgs::OccupancyGrid* occupancy_grid);

}  // namespace cartographer_ros

#endif  // CARTOGRAHPER_ROS_OCCUPANCY_GRID_H_
