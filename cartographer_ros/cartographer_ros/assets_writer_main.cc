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
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/split_string.h"
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
DEFINE_string(bag_filenames, "",
              "Bags to process, must be in the same order as the trajectories "
              "in 'pose_graph_filename'.");
DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read and use the transforms from the bag.");

namespace cartographer_ros {
namespace {

constexpr char kTfStaticTopic[] = "/tf_static";
namespace carto = ::cartographer;

template <typename T>
std::unique_ptr<carto::io::PointsBatch> HandleMessage(
    const T& message, const string& tracking_frame,
    const tf2_ros::Buffer& tf_buffer,
    const carto::transform::TransformInterpolationBuffer&
        transform_interpolation_buffer) {
  const carto::common::Time start_time = FromRos(message.header.stamp);

  auto points_batch = carto::common::make_unique<carto::io::PointsBatch>();
  points_batch->start_time = start_time;
  points_batch->frame_id = message.header.frame_id;

  carto::sensor::PointCloudWithIntensities point_cloud =
      ToPointCloudWithIntensities(message);
  CHECK_EQ(point_cloud.intensities.size(), point_cloud.points.size());
  CHECK_EQ(point_cloud.offset_seconds.size(), point_cloud.points.size());

  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    const carto::common::Time time =
        start_time + carto::common::FromSeconds(point_cloud.offset_seconds[i]);
    if (!transform_interpolation_buffer.Has(time)) {
      continue;
    }
    const carto::transform::Rigid3d tracking_to_map =
        transform_interpolation_buffer.Lookup(time);
    const carto::transform::Rigid3d sensor_to_tracking =
        ToRigid3d(tf_buffer.lookupTransform(
            tracking_frame, message.header.frame_id, ToRos(time)));
    const carto::transform::Rigid3f sensor_to_map =
        (tracking_to_map * sensor_to_tracking).cast<float>();
    points_batch->points.push_back(sensor_to_map * point_cloud.points[i]);
    points_batch->intensities.push_back(point_cloud.intensities[i]);
    // We use the last transform for the origin, which is approximately correct.
    points_batch->origin = sensor_to_map * Eigen::Vector3f::Zero();
  }
  if (points_batch->points.empty()) {
    return nullptr;
  }
  return points_batch;
}

void Run(const string& pose_graph_filename,
         const std::vector<string>& bag_filenames,
         const string& configuration_directory,
         const string& configuration_basename, const string& urdf_filename) {
  auto file_resolver =
      carto::common::make_unique<carto::common::ConfigurationFileResolver>(
          std::vector<string>{configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  carto::io::ProtoStreamReader reader(pose_graph_filename);
  carto::mapping::proto::SparsePoseGraph pose_graph_proto;
  CHECK(reader.ReadProto(&pose_graph_proto));
  CHECK_EQ(pose_graph_proto.trajectory_size(), bag_filenames.size())
      << "Pose graphs contains " << pose_graph_proto.trajectory_size()
      << " trajectories while " << bag_filenames.size()
      << " bags were provided. This tool requires one bag for each "
         "trajectory in the same order as the correponding trajectories in the "
         "pose graph proto.";

  const auto file_writer_factory = [](const string& filename) {
    return carto::common::make_unique<carto::io::StreamFileWriter>(filename);
  };

  // This vector must outlive the pipeline.
  std::vector<::cartographer::mapping::proto::Trajectory> all_trajectories(
      pose_graph_proto.trajectory().begin(),
      pose_graph_proto.trajectory().end());

  carto::io::PointsProcessorPipelineBuilder builder;
  carto::io::RegisterBuiltInPointsProcessors(all_trajectories,
                                             file_writer_factory, &builder);
  std::vector<std::unique_ptr<carto::io::PointsProcessor>> pipeline =
      builder.CreatePipeline(
          lua_parameter_dictionary.GetDictionary("pipeline").get());

  const string tracking_frame =
      lua_parameter_dictionary.GetString("tracking_frame");
  do {
    for (size_t trajectory_id = 0; trajectory_id < bag_filenames.size();
         ++trajectory_id) {
      const carto::mapping::proto::Trajectory& trajectory_proto =
          pose_graph_proto.trajectory(trajectory_id);
      const string& bag_filename = bag_filenames[trajectory_id];
      LOG(INFO) << "Processing " << bag_filename << "...";
      if (trajectory_proto.node_size() == 0) {
        continue;
      }
      tf2_ros::Buffer tf_buffer;
      if (!urdf_filename.empty()) {
        ReadStaticTransformsFromUrdf(urdf_filename, &tf_buffer);
      }

      const carto::transform::TransformInterpolationBuffer
          transform_interpolation_buffer(trajectory_proto);
      rosbag::Bag bag;
      bag.open(bag_filename, rosbag::bagmode::Read);
      rosbag::View view(bag);
      const ::ros::Time begin_time = view.getBeginTime();
      const double duration_in_seconds =
          (view.getEndTime() - begin_time).toSec();

      // We need to keep 'tf_buffer' small because it becomes very inefficient
      // otherwise. We make sure that tf_messages are published before any data
      // messages, so that tf lookups always work.
      std::deque<rosbag::MessageInstance> delayed_messages;
      // We publish tf messages one second earlier than other messages. Under
      // the assumption of higher frequency tf this should ensure that tf can
      // always interpolate.
      const ::ros::Duration kDelay(1.);
      for (const rosbag::MessageInstance& message : view) {
        if (FLAGS_use_bag_transforms && message.isType<tf2_msgs::TFMessage>()) {
          auto tf_message = message.instantiate<tf2_msgs::TFMessage>();
          for (const auto& transform : tf_message->transforms) {
            try {
              tf_buffer.setTransform(transform, "unused_authority",
                                     message.getTopic() == kTfStaticTopic);
            } catch (const tf2::TransformException& ex) {
              LOG(WARNING) << ex.what();
            }
          }
        }

        while (!delayed_messages.empty() && delayed_messages.front().getTime() <
                                                message.getTime() - kDelay) {
          const rosbag::MessageInstance& delayed_message =
              delayed_messages.front();

          std::unique_ptr<carto::io::PointsBatch> points_batch;
          if (delayed_message.isType<sensor_msgs::PointCloud2>()) {
            points_batch = HandleMessage(
                *delayed_message.instantiate<sensor_msgs::PointCloud2>(),
                tracking_frame, tf_buffer, transform_interpolation_buffer);
          } else if (delayed_message
                         .isType<sensor_msgs::MultiEchoLaserScan>()) {
            points_batch = HandleMessage(
                *delayed_message.instantiate<sensor_msgs::MultiEchoLaserScan>(),
                tracking_frame, tf_buffer, transform_interpolation_buffer);
          } else if (delayed_message.isType<sensor_msgs::LaserScan>()) {
            points_batch = HandleMessage(
                *delayed_message.instantiate<sensor_msgs::LaserScan>(),
                tracking_frame, tf_buffer, transform_interpolation_buffer);
          }
          if (points_batch != nullptr) {
            points_batch->trajectory_id = trajectory_id;
            pipeline.back()->Process(std::move(points_batch));
          }
          delayed_messages.pop_front();
        }
        delayed_messages.push_back(message);
        LOG_EVERY_N(INFO, 100000)
            << "Processed " << (message.getTime() - begin_time).toSec()
            << " of " << duration_in_seconds << " bag time seconds...";
      }
      bag.close();
    }
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
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
  CHECK(!FLAGS_pose_graph_filename.empty())
      << "-pose_graph_filename is missing.";

  ::cartographer_ros::Run(
      FLAGS_pose_graph_filename,
      cartographer_ros::SplitString(FLAGS_bag_filenames, ','),
      FLAGS_configuration_directory, FLAGS_configuration_basename,
      FLAGS_urdf_filename);
}
