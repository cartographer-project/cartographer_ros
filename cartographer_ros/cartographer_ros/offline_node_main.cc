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

#include <csignal>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/urdf_reader.h"
#include "gflags/gflags.h"
#include "ros/callback_queue.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosgraph_msgs/Clock.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "urdf/model.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(bag_filenames, "", "Comma-separated list of bags to process.");
DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish the transforms from the bag.");

namespace cartographer_ros {
namespace {

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "tf";
constexpr int kLatestOnlyPublisherQueueSize = 1;

volatile std::sig_atomic_t sigint_triggered = 0;

void SigintHandler(int) { sigint_triggered = 1; }

std::vector<string> SplitString(const string& input, const char delimiter) {
  std::stringstream stream(input);
  string token;
  std::vector<string> tokens;
  while (std::getline(stream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// TODO(hrapp): This is duplicated in node_main.cc. Pull out into a config
// unit.
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

void Run(const std::vector<string>& bag_filenames) {
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) = LoadOptions();

  tf2_ros::Buffer tf_buffer;

  std::vector<geometry_msgs::TransformStamped> urdf_transforms;
  if (!FLAGS_urdf_filename.empty()) {
    urdf_transforms =
        ReadStaticTransformsFromUrdf(FLAGS_urdf_filename, &tf_buffer);
  }

  tf_buffer.setUsingDedicatedThread(true);

  // Since we preload the transform buffer, we should never have to wait for a
  // transform. When we finish processing the bag, we will simply drop any
  // remaining sensor data that cannot be transformed due to missing transforms.
  node_options.lookup_transform_timeout_sec = 0.;
  Node node(node_options, &tf_buffer);

  std::unordered_set<string> expected_sensor_ids;
  const auto check_insert = [&expected_sensor_ids, &node](const string& topic) {
    CHECK(expected_sensor_ids.insert(node.node_handle()->resolveName(topic))
              .second);
  };

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  if (trajectory_options.use_laser_scan) {
    check_insert(kLaserScanTopic);
  }
  if (trajectory_options.use_multi_echo_laser_scan) {
    check_insert(kMultiEchoLaserScanTopic);
  }

  // For 3D SLAM, subscribe to all point clouds topics.
  if (trajectory_options.num_point_clouds > 0) {
    for (int i = 0; i < trajectory_options.num_point_clouds; ++i) {
      // TODO(hrapp): This code is duplicated in places. Pull out a method.
      string topic = kPointCloud2Topic;
      if (trajectory_options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      check_insert(topic);
    }
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options.map_builder_options.use_trajectory_builder_3d() ||
      (node_options.map_builder_options.use_trajectory_builder_2d() &&
       trajectory_options.trajectory_builder_options
           .trajectory_builder_2d_options()
           .use_imu_data())) {
    check_insert(kImuTopic);
  }

  // For both 2D and 3D SLAM, odometry is optional.
  if (trajectory_options.use_odometry) {
    check_insert(kOdometryTopic);
  }

  ::ros::Publisher tf_publisher =
      node.node_handle()->advertise<tf2_msgs::TFMessage>(
          kTfTopic, kLatestOnlyPublisherQueueSize);

  ::tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

  ::ros::Publisher clock_publisher =
      node.node_handle()->advertise<rosgraph_msgs::Clock>(
          kClockTopic, kLatestOnlyPublisherQueueSize);

  if (urdf_transforms.size() > 0) {
    static_tf_broadcaster.sendTransform(urdf_transforms);
  }

  for (const string& bag_filename : bag_filenames) {
    if (sigint_triggered) {
      break;
    }

    const int trajectory_id = node.map_builder_bridge()->AddTrajectory(
        expected_sensor_ids, trajectory_options);

    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);
    rosbag::View view(bag);
    const ::ros::Time begin_time = view.getBeginTime();
    const double duration_in_seconds = (view.getEndTime() - begin_time).toSec();

    // We make sure that tf_messages are published before any data messages, so
    // that tf lookups always work and that tf_buffer has a small cache size -
    // because it gets very inefficient with a large one.
    std::deque<rosbag::MessageInstance> delayed_messages;
    for (const rosbag::MessageInstance& msg : view) {
      if (sigint_triggered) {
        break;
      }

      if (FLAGS_use_bag_transforms && msg.isType<tf2_msgs::TFMessage>()) {
        auto tf_message = msg.instantiate<tf2_msgs::TFMessage>();
        tf_publisher.publish(tf_message);

        for (const auto& transform : tf_message->transforms) {
          try {
            tf_buffer.setTransform(transform, "unused_authority",
                                   msg.getTopic() == kTfStaticTopic);
          } catch (const tf2::TransformException& ex) {
            LOG(WARNING) << ex.what();
          }
        }
      }

      while (!delayed_messages.empty() &&
             delayed_messages.front().getTime() <
                 msg.getTime() + ::ros::Duration(1.)) {
        const rosbag::MessageInstance& delayed_msg = delayed_messages.front();
        const string topic = node.node_handle()->resolveName(
            delayed_msg.getTopic(), false /* resolve */);
        if (delayed_msg.isType<sensor_msgs::LaserScan>()) {
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleLaserScanMessage(
                  topic, delayed_msg.instantiate<sensor_msgs::LaserScan>());
        }
        if (delayed_msg.isType<sensor_msgs::MultiEchoLaserScan>()) {
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleMultiEchoLaserScanMessage(
                  topic,
                  delayed_msg.instantiate<sensor_msgs::MultiEchoLaserScan>());
        }
        if (delayed_msg.isType<sensor_msgs::PointCloud2>()) {
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandlePointCloud2Message(
                  topic, delayed_msg.instantiate<sensor_msgs::PointCloud2>());
        }
        if (delayed_msg.isType<sensor_msgs::Imu>()) {
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleImuMessage(topic,
                                 delayed_msg.instantiate<sensor_msgs::Imu>());
        }
        if (delayed_msg.isType<nav_msgs::Odometry>()) {
          node.map_builder_bridge()
              ->sensor_bridge(trajectory_id)
              ->HandleOdometryMessage(
                  topic, delayed_msg.instantiate<nav_msgs::Odometry>());
        }
        rosgraph_msgs::Clock clock;
        clock.clock = delayed_msg.getTime();
        clock_publisher.publish(clock);

        ::ros::spinOnce();

        LOG_EVERY_N(INFO, 100000)
            << "Processed " << (delayed_msg.getTime() - begin_time).toSec()
            << " of " << duration_in_seconds << " bag time seconds...";

        delayed_messages.pop_front();
      }

      const string topic =
          node.node_handle()->resolveName(msg.getTopic(), false /* resolve */);
      if (expected_sensor_ids.count(topic) == 0) {
        continue;
      }
      delayed_messages.push_back(msg);
    }

    bag.close();
    node.map_builder_bridge()->FinishTrajectory(trajectory_id);
  }

  node.map_builder_bridge()->SerializeState(bag_filenames.front());
  node.map_builder_bridge()->WriteAssets(bag_filenames.front());
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
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";

  std::signal(SIGINT, &::cartographer_ros::SigintHandler);
  ::ros::init(argc, argv, "cartographer_offline_node",
              ::ros::init_options::NoSigintHandler);
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run(
      cartographer_ros::SplitString(FLAGS_bag_filenames, ','));

  ::ros::shutdown();
}
