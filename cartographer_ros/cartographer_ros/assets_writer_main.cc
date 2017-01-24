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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/bag_reader.h"
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
#include "tf2_msgs/TFMessage.h"
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

carto::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::PointCloud2::ConstPtr& message) {
  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
  pcl::fromROSMsg(*message, pcl_point_cloud);
  carto::sensor::PointCloudWithIntensities point_cloud;

  // TODO(hrapp): How to get reflectivities from PCL?
  for (const auto& point : pcl_point_cloud) {
    point_cloud.points.emplace_back(point.x, point.y, point.z);
    point_cloud.intensities.push_back(1.);
  }
  return point_cloud;
}

carto::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& message) {
  return carto::sensor::ToPointCloudWithIntensities(ToCartographer(*message));
}

carto::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::LaserScan::ConstPtr& message) {
  return carto::sensor::ToPointCloudWithIntensities(ToCartographer(*message));
}

template <typename T>
void HandleMessage(
    const T& message, const string& tracking_frame,
    const tf2_ros::Buffer& tf_buffer,
    const carto::transform::TransformInterpolationBuffer&
        transform_interpolation_buffer,
    const std::vector<std::unique_ptr<carto::io::PointsProcessor>>& pipeline) {
  const carto::common::Time time = FromRos(message->header.stamp);
  if (!transform_interpolation_buffer.Has(time)) {
    return;
  }

  const carto::transform::Rigid3d tracking_to_map =
      transform_interpolation_buffer.Lookup(time);
  const carto::transform::Rigid3d sensor_to_tracking =
      ToRigid3d(tf_buffer.lookupTransform(
          tracking_frame, message->header.frame_id, message->header.stamp));
  const carto::transform::Rigid3f sensor_to_map =
      (tracking_to_map * sensor_to_tracking).cast<float>();

  auto batch = carto::common::make_unique<carto::io::PointsBatch>();
  batch->time = time;
  batch->origin = sensor_to_map * Eigen::Vector3f::Zero();
  batch->frame_id = message->header.frame_id;

  carto::sensor::PointCloudWithIntensities point_cloud =
      ToPointCloudWithIntensities(message);
  CHECK(point_cloud.intensities.size() == point_cloud.points.size());

  for (int i = 0; i < point_cloud.points.size(); ++i) {
    batch->points.push_back(sensor_to_map * point_cloud.points[i]);
    batch->intensities.push_back(point_cloud.intensities[i]);
  }
  pipeline.back()->Process(std::move(batch));
}

void Run(const string& trajectory_filename, const string& bag_filename,
         const string& configuration_directory,
         const string& configuration_basename, const string& urdf_filename) {
  auto file_resolver =
      carto::common::make_unique<carto::common::ConfigurationFileResolver>(
          std::vector<string>{configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  std::ifstream stream(trajectory_filename.c_str());
  carto::mapping::proto::Trajectory trajectory_proto;
  CHECK(trajectory_proto.ParseFromIstream(&stream));

  const auto file_writer_factory = [](const string& filename) {
    return carto::common::make_unique<carto::io::StreamFileWriter>(filename);
  };

  carto::io::PointsProcessorPipelineBuilder builder;
  carto::io::RegisterBuiltInPointsProcessors(trajectory_proto,
                                             file_writer_factory, &builder);
  std::vector<std::unique_ptr<carto::io::PointsProcessor>> pipeline =
      builder.CreatePipeline(
          lua_parameter_dictionary.GetDictionary("pipeline").get());

  auto tf_buffer = ::cartographer::common::make_unique<tf2_ros::Buffer>();
  if (!urdf_filename.empty()) {
    ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer.get());
  } else {
    LOG(INFO) << "Pre-loading transforms from bag...";
    tf_buffer = ReadTransformsFromBag(bag_filename);
  }

  const string tracking_frame =
      lua_parameter_dictionary.GetString("tracking_frame");
  const auto transform_interpolation_buffer =
      carto::transform::TransformInterpolationBuffer::FromTrajectory(
          trajectory_proto);
  LOG(INFO) << "Processing pipeline...";
  do {
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);
    rosbag::View view(bag);
    const ::ros::Time begin_time = view.getBeginTime();
    const double duration_in_seconds = (view.getEndTime() - begin_time).toSec();

    for (const rosbag::MessageInstance& message : view) {
      if (message.isType<sensor_msgs::PointCloud2>()) {
        HandleMessage(message.instantiate<sensor_msgs::PointCloud2>(),
                      tracking_frame, *tf_buffer,
                      *transform_interpolation_buffer, pipeline);
      }
      if (message.isType<sensor_msgs::MultiEchoLaserScan>()) {
        HandleMessage(message.instantiate<sensor_msgs::MultiEchoLaserScan>(),
                      tracking_frame, *tf_buffer,
                      *transform_interpolation_buffer, pipeline);
      }
      if (message.isType<sensor_msgs::LaserScan>()) {
        HandleMessage(message.instantiate<sensor_msgs::LaserScan>(),
                      tracking_frame, *tf_buffer,
                      *transform_interpolation_buffer, pipeline);
      }
      LOG_EVERY_N(INFO, 100000)
          << "Processed " << (message.getTime() - begin_time).toSec() << " of "
          << duration_in_seconds << " bag time seconds...";
    }
    bag.close();
  } while (pipeline.back()->Flush() ==
           carto::io::PointsProcessor::FlushResult::kRestartStream);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  CHECK(!FLAGS_trajectory_filename.empty())
      << "-trajectory_filename is missing.";

  ::cartographer_ros::Run(FLAGS_trajectory_filename, FLAGS_bag_filename,
                          FLAGS_configuration_directory,
                          FLAGS_configuration_basename, FLAGS_urdf_filename);
}
