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

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

#ifndef CARTOGRAPHER_ROS_ASSETS_WRITER_H_
#define CARTOGRAPHER_ROS_ASSETS_WRITER_H_

namespace cartographer_ros {

class AssetsWriter {
 public:
  // Configures a point processing pipeline.
  AssetsWriter(const std::string& pose_graph_filename,
               const std::vector<std::string>& bag_filenames,
               const std::string& configuration_directory,
               const std::string& configuration_basename,
               const std::string& urdf_filename,
               const std::string& output_file_prefix,
               const bool use_bag_transforms);

  // Pushes the points from the bag through the point processing pipeline.
  void Run();

 private:
  std::vector<std::string> bag_filenames_;
  std::string urdf_filename_;
  bool use_bag_transforms_;
  std::vector<::cartographer::mapping::proto::Trajectory> all_trajectories_;
  ::cartographer::mapping::proto::PoseGraph pose_graph_;
  ::cartographer::mapping::proto::AllTrajectoryBuilderOptions
      trajectory_options_;
  std::unique_ptr<::cartographer::common::LuaParameterDictionary>
      lua_parameter_dictionary_;
  std::unique_ptr<::cartographer::io::PointsProcessorPipelineBuilder>
      point_pipeline_builder_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_ASSETS_WRITER_H_
