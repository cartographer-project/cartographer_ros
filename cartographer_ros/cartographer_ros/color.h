/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_ROS_COLOR_MANAGER_H_
#define CARTOGRAPHER_ROS_COLOR_MANAGER_H_

#include <std_msgs/ColorRGBA.h>

namespace cartographer_ros {

// A function for on-demand generation of a color palette, with every two
// direct successors having large contrast.
::std_msgs::ColorRGBA GetColor(int id);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_COLOR_MANAGER_H_
