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
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
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
    pose_graph_filename, "",
    "Proto containing the pose graph written by /write_assets service.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read and use the transforms from the bag.");

namespace cartographer_ros {
namespace {

constexpr char kTfStaticTopic[] = "/tf_static";
namespace carto = ::cartographer;

// TODO(hrapp): We discovered that using tf_buffer with a large CACHE
// is very inefficient. Switch asset writer to use our own
// TransformInterpolationBuffer.
void ReadTransformsFromBag(const string& bag_filename,
                           tf2_ros::Buffer* const tf_buffer) {
  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag);

  const ::ros::Time begin_time = view.getBeginTime();
  const double duration_in_seconds = (view.getEndTime() - begin_time).toSec();
  for (const rosbag::MessageInstance& msg : view) {
    if (msg.isType<tf2_msgs::TFMessage>()) {
      const auto tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
      for (const auto& transform : tf_msg->transforms) {
        try {
          // TODO(damonkohler): Handle topic remapping.
          tf_buffer->setTransform(transform, "unused_authority",
                                  msg.getTopic() == kTfStaticTopic);
        } catch (const tf2::TransformException& ex) {
          LOG(WARNING) << ex.what();
        }
      }
    }
    LOG_EVERY_N(INFO, 100000)
        << "Processed " << (msg.getTime() - begin_time).toSec() << " of "
        << duration_in_seconds << " bag time seconds...";
  }

  bag.close();
}

template <typename T>
void HandleMessage(
    const T& message, const string& tracking_frame,
    const tf2_ros::Buffer& tf_buffer,
    const carto::transform::TransformInterpolationBuffer&
        transform_interpolation_buffer,
    const std::vector<std::unique_ptr<carto::io::PointsProcessor>>& pipeline) {
  const carto::common::Time time = FromRos(message.header.stamp);
  if (!transform_interpolation_buffer.Has(time)) {
    return;
  }

  const carto::transform::Rigid3d tracking_to_map =
      transform_interpolation_buffer.Lookup(time);
  const carto::transform::Rigid3d sensor_to_tracking =
      ToRigid3d(tf_buffer.lookupTransform(
          tracking_frame, message.header.frame_id, message.header.stamp));
  const carto::transform::Rigid3f sensor_to_map =
      (tracking_to_map * sensor_to_tracking).cast<float>();

  auto batch = carto::common::make_unique<carto::io::PointsBatch>();
  batch->time = time;
  batch->origin = sensor_to_map * Eigen::Vector3f::Zero();
  batch->frame_id = message.header.frame_id;

  carto::sensor::PointCloudWithIntensities point_cloud =
      ToPointCloudWithIntensities(message);
  CHECK(point_cloud.intensities.size() == point_cloud.points.size());

  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    batch->points.push_back(sensor_to_map * point_cloud.points[i]);
    batch->intensities.push_back(point_cloud.intensities[i]);
  }
  pipeline.back()->Process(std::move(batch));
}

void Run(const string& pose_graph_filename, const string& bag_filename,
         const string& configuration_directory,
         const string& configuration_basename, const string& urdf_filename) {
  auto file_resolver =
      carto::common::make_unique<carto::common::ConfigurationFileResolver>(
          std::vector<string>{configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  std::ifstream stream(pose_graph_filename.c_str());
  carto::mapping::proto::SparsePoseGraph pose_graph_proto;
  CHECK(pose_graph_proto.ParseFromIstream(&stream));
  CHECK_EQ(pose_graph_proto.trajectory_size(), 1)
      << "Only pose graphs containing a single trajectory are supported.";
  const carto::mapping::proto::Trajectory& trajectory_proto =
      pose_graph_proto.trajectory(0);
  CHECK_GT(trajectory_proto.node_size(), 0)
      << "Trajectory does not contain any nodes.";

  const auto file_writer_factory = [](const string& filename) {
    return carto::common::make_unique<carto::io::StreamFileWriter>(filename);
  };

  carto::io::PointsProcessorPipelineBuilder builder;
  carto::io::RegisterBuiltInPointsProcessors(trajectory_proto,
                                             file_writer_factory, &builder);
  std::vector<std::unique_ptr<carto::io::PointsProcessor>> pipeline =
      builder.CreatePipeline(
          lua_parameter_dictionary.GetDictionary("pipeline").get());

  tf2_ros::Buffer tf_buffer(::ros::DURATION_MAX);

  if (FLAGS_use_bag_transforms) {
    LOG(INFO) << "Pre-loading transforms from bag...";
    ReadTransformsFromBag(bag_filename, &tf_buffer);
  }

  if (!urdf_filename.empty()) {
    ReadStaticTransformsFromUrdf(urdf_filename, &tf_buffer);
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
        HandleMessage(*message.instantiate<sensor_msgs::PointCloud2>(),
                      tracking_frame, tf_buffer,
                      *transform_interpolation_buffer, pipeline);
      }
      if (message.isType<sensor_msgs::MultiEchoLaserScan>()) {
        HandleMessage(*message.instantiate<sensor_msgs::MultiEchoLaserScan>(),
                      tracking_frame, tf_buffer,
                      *transform_interpolation_buffer, pipeline);
      }
      if (message.isType<sensor_msgs::LaserScan>()) {
        HandleMessage(*message.instantiate<sensor_msgs::LaserScan>(),
                      tracking_frame, tf_buffer,
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
  CHECK(!FLAGS_pose_graph_filename.empty())
      << "-pose_graph_filename is missing.";

  ::cartographer_ros::Run(FLAGS_pose_graph_filename, FLAGS_bag_filename,
                          FLAGS_configuration_directory,
                          FLAGS_configuration_basename, FLAGS_urdf_filename);
}
