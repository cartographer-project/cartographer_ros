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

#include <string>
#include <vector>

#ifndef CARTOGRAPHER_ROS_ASSETS_WRITER_H_
#define CARTOGRAPHER_ROS_ASSETS_WRITER_H_

namespace cartographer_ros {

// Configures a point processing pipeline and pushes the points from the bag
// through it.
void RunAssetsWriterPipeline(const std::string& pose_graph_filename,
                             const std::vector<std::string>& bag_filenames,
                             const std::string& configuration_directory,
                             const std::string& configuration_basename,
                             const std::string& urdf_filename,
                             const std::string& output_file_prefix,
                             bool use_bag_transforms);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_ASSETS_WRITER_H_
