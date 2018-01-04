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

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/offline_node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/split_string.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(bag_filenames, "", "Comma-separated list of bags to process.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

  ::ros::init(argc, argv, "cartographer_offline_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  cartographer_ros::NodeOptions node_options;
  cartographer_ros::TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) = cartographer_ros::LoadOptions(
      FLAGS_configuration_directory, FLAGS_configuration_basename);

  // Since we preload the transform buffer, we should never have to wait for a
  // transform. When we finish processing the bag, we will simply drop any
  // remaining sensor data that cannot be transformed due to missing transforms.
  node_options.lookup_transform_timeout_sec = 0.;

  auto map_builder =
      cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
          node_options.map_builder_options);

  cartographer_ros::RunOfflineNode(
      std::move(map_builder), node_options, trajectory_options,
      cartographer_ros::SplitString(FLAGS_bag_filenames, ','));

  ::ros::shutdown();
}
