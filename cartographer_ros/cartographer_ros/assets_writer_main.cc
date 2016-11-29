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

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/buffer.h"
#include "urdf/model.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");
DEFINE_string(bag_filename, "", "Bag to process.");
DEFINE_string(
    trajectory_filename, "",
    "Proto containing the trajectory written by /finish_trajectory service.");

namespace cartographer_ros {
namespace {

namespace carto = ::cartographer;

void Run(const string& trajectory_filename, const string& bag_filename,
         const string& configuration_directory,
         const string& configuration_basename, const string& urdf_filename) {
  ::tf2_ros::Buffer buffer;
  ReadStaticTransformsFromUrdf(urdf_filename, &buffer);

  std::ifstream stream(trajectory_filename.c_str());
  carto::mapping::proto::Trajectory trajectory_proto;
  CHECK(trajectory_proto.ParseFromIstream(&stream));

  auto transform_interpolation_buffer =
      carto::transform::TransformInterpolationBuffer::FromTrajectory(
          trajectory_proto);

  carto::io::PointsProcessorPipelineBuilder builder;
  carto::io::RegisterBuiltInPointsProcessors(trajectory_proto, &builder);

  auto file_resolver =
      carto::common::make_unique<carto::common::ConfigurationFileResolver>(
          std::vector<string>{configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));
  const string tracking_frame =
      lua_parameter_dictionary.GetString("tracking_frame");

  std::vector<std::unique_ptr<carto::io::PointsProcessor>> pipeline =
      builder.CreatePipeline(
          lua_parameter_dictionary.GetDictionary("pipeline").get());

  do {
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);
    // TODO(hrapp): Also parse tf messages and keep the 'buffer' updated as to
    // support non-rigid sensor configurations.
    rosbag::View view(
        bag,
        rosbag::TypeQuery(std::vector<std::string>{"sensor_msgs/PointCloud2"}));
    const ros::Time bag_start_timestamp = view.getBeginTime();
    const ros::Time bag_end_timestamp = view.getEndTime();

    for (const rosbag::MessageInstance& m : view) {
      auto point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (point_cloud_msg != nullptr) {
        const carto::common::Time time = FromRos(point_cloud_msg->header.stamp);
        if (!transform_interpolation_buffer->Has(time)) {
          continue;
        }
        carto::transform::Rigid3d tracking_to_map =
            transform_interpolation_buffer->Lookup(time);
        pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
        pcl::fromROSMsg(*point_cloud_msg, pcl_point_cloud);

        const carto::transform::Rigid3d sensor_to_tracking =
            ToRigid3d(buffer.lookupTransform(tracking_frame,
                                             point_cloud_msg->header.frame_id,
                                             point_cloud_msg->header.stamp));

        const carto::transform::Rigid3f sensor_to_map =
            (tracking_to_map * sensor_to_tracking).cast<float>();

        auto batch = carto::common::make_unique<carto::io::PointsBatch>();
        batch->time = time;
        batch->origin = sensor_to_map * Eigen::Vector3f::Zero();
        batch->frame_id = point_cloud_msg->header.frame_id;

        for (const auto& point : pcl_point_cloud) {
          batch->points.push_back(sensor_to_map *
                                  Eigen::Vector3f(point.x, point.y, point.z));
        }
        pipeline.back()->Process(std::move(batch));
      }
      ::ros::Time ros_time = m.getTime();
      LOG_EVERY_N(INFO, 1000)
          << "Processed " << (ros_time - bag_start_timestamp).toSec() << " of "
          << (bag_end_timestamp - bag_start_timestamp).toSec()
          << " bag time seconds.";
    }
    bag.close();
  } while (pipeline.back()->Flush() ==
           carto::io::PointsProcessor::FlushResult::kRestartStream);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  CHECK(!FLAGS_trajectory_filename.empty())
      << "-trajectory_filename is missing.";
  CHECK(!FLAGS_urdf_filename.empty()) << "-urdf_filename is missing.";

  ::cartographer_ros::Run(FLAGS_trajectory_filename, FLAGS_bag_filename,
                          FLAGS_configuration_directory,
                          FLAGS_configuration_basename, FLAGS_urdf_filename);
}
