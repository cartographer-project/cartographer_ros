/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer_ros/offline_node.h"

#include <errno.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <chrono>

#include "cartographer_ros/node.h"
#include "cartographer_ros/playable_bag.h"
#include "cartographer_ros/split_string.h"
#include "cartographer_ros/urdf_reader.h"
#include "ros/callback_queue.h"
#include "rosgraph_msgs/Clock.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "urdf/model.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basenames, "",
              "Basenames, i.e. not containing any directory prefix, of the "
              "configuration files for each trajectory.");
DEFINE_string(
    bag_filenames, "",
    "Comma-separated list of bags to process. One bag per trajectory.");
DEFINE_string(urdf_filenames, "",
              "One or more URDF files that contains static links for your "
              "sensor configuration.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish transforms from bags.");
DEFINE_string(pbstream_filename, "",
              "If non-empty, filename of a pbstream to load.");
DEFINE_bool(keep_running, false,
            "Keep running the offline node after all messages from the bag "
            "have been processed.");
DEFINE_string(sensor_topics, "",
              "Optional list of sensor topics for each trajectory. A single "
              "trajectory's topics are delimited with colons, while different "
              "trajectories are delimited with commas. If a blank list is "
              "given for a trajectory, the default topics are used.");

namespace cartographer_ros {

constexpr char kClockTopic[] = "clock";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;
// We publish tf messages one second earlier than other messages. Under
// the assumption of higher frequency tf this should ensure that tf can
// always interpolate.
const ::ros::Duration kDelay = ::ros::Duration(1.0);

void RunOfflineNode(const MapBuilderFactory& map_builder_factory) {
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basenames.empty())
      << "-configuration_basenames is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
  const auto bag_filenames =
      cartographer_ros::SplitString(FLAGS_bag_filenames, ',');
  cartographer_ros::NodeOptions node_options;
  const auto configuration_basenames =
      cartographer_ros::SplitString(FLAGS_configuration_basenames, ',');
  TrajectoryOptions first_trajectory_options;
  std::tie(node_options, first_trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, configuration_basenames.at(0));

  std::vector<TrajectoryOptions> trajectory_options;
  if (configuration_basenames.size() == 1) {
    trajectory_options = std::vector<TrajectoryOptions>{
        bag_filenames.size(), first_trajectory_options};
  } else {
    for (const std::string& configuration_basename : configuration_basenames) {
      TrajectoryOptions current_trajectory_options;
      std::tie(std::ignore, current_trajectory_options) =
          LoadOptions(FLAGS_configuration_directory, configuration_basename);
      trajectory_options.push_back(std::move(current_trajectory_options));
    }
  }
  CHECK_EQ(trajectory_options.size(), bag_filenames.size());

  // Since we preload the transform buffer, we should never have to wait for a
  // transform. When we finish processing the bag, we will simply drop any
  // remaining sensor data that cannot be transformed due to missing transforms.
  node_options.lookup_transform_timeout_sec = 0.;

  auto map_builder = map_builder_factory(node_options.map_builder_options);

  const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();

  tf2_ros::Buffer tf_buffer;

  std::vector<geometry_msgs::TransformStamped> urdf_transforms;
  for (const std::string& urdf_filename :
       cartographer_ros::SplitString(FLAGS_urdf_filenames, ',')) {
    const auto current_urdf_transforms =
        ReadStaticTransformsFromUrdf(urdf_filename, &tf_buffer);
    urdf_transforms.insert(urdf_transforms.end(),
                           current_urdf_transforms.begin(),
                           current_urdf_transforms.end());
  }

  tf_buffer.setUsingDedicatedThread(true);

  Node node(node_options, std::move(map_builder), &tf_buffer);
  if (!FLAGS_pbstream_filename.empty()) {
    // TODO(jihoonl): LoadMap should be replaced by some better deserialization
    // of full SLAM state as non-frozen trajectories once possible
    node.LoadMap(FLAGS_pbstream_filename);
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

  ros::AsyncSpinner async_spinner(kSingleThreaded);
  async_spinner.start();
  rosgraph_msgs::Clock clock;
  auto clock_republish_timer = node.node_handle()->createWallTimer(
      ::ros::WallDuration(kClockPublishFrequencySec),
      [&clock_publisher, &clock](const ::ros::WallTimerEvent&) {
        clock_publisher.publish(clock);
      },
      false /* oneshot */, false /* autostart */);

  std::vector<std::string> specified_sensor_topics =
      cartographer_ros::SplitString(FLAGS_sensor_topics, ',');

  if (specified_sensor_topics.empty()) {
    specified_sensor_topics =
        std::vector<std::string>(bag_filenames.size(), std::string());
  } else if (specified_sensor_topics.size() == 1) {
    specified_sensor_topics.insert(specified_sensor_topics.end(),
                                   bag_filenames.size() - 1,
                                   specified_sensor_topics.at(0));
  }
  CHECK_EQ(specified_sensor_topics.size(), bag_filenames.size());

  std::unordered_map<int, std::unordered_set<std::string>>
      expected_sensor_topics;
  PlayableBagMultiplexer playable_bag_multiplexer;
  for (size_t current_bag_index = 0; current_bag_index < bag_filenames.size();
       ++current_bag_index) {
    const std::string& bag_filename = bag_filenames.at(current_bag_index);
    if (!::ros::ok()) {
      return;
    }
    std::unordered_set<std::string> current_expected_sensor_topics;
    if (specified_sensor_topics.at(current_bag_index).empty()) {
      for (const std::string& topic : node.ComputeDefaultTopics(
               trajectory_options.at(current_bag_index))) {
        CHECK(current_expected_sensor_topics
                  .insert(node.node_handle()->resolveName(topic))
                  .second);
      }
    } else {
      for (const std::string& topic : cartographer_ros::SplitString(
               specified_sensor_topics.at(current_bag_index), ':')) {
        CHECK(current_expected_sensor_topics
                  .insert(node.node_handle()->resolveName(topic))
                  .second);
      }
    }
    const int trajectory_id =
        node.AddOfflineTrajectory(current_expected_sensor_topics,
                                  trajectory_options.at(current_bag_index));
    CHECK(expected_sensor_topics
              .emplace(std::piecewise_construct,
                       std::forward_as_tuple(trajectory_id),
                       std::forward_as_tuple(current_expected_sensor_topics))
              .second);

    playable_bag_multiplexer.AddPlayableBag(PlayableBag(
        bag_filename, trajectory_id, ros::TIME_MIN, ros::TIME_MAX, kDelay,
        [&](const rosbag::MessageInstance& msg) {
          if (msg.isType<tf2_msgs::TFMessage>()) {
            if (FLAGS_use_bag_transforms) {
              const auto tf_message = msg.instantiate<tf2_msgs::TFMessage>();
              tf_publisher.publish(tf_message);

              for (const auto& transform : tf_message->transforms) {
                try {
                  // We need to keep 'tf_buffer' small because it becomes very
                  // inefficient otherwise. We make sure that tf_messages are
                  // published before any data messages, so that tf lookups
                  // always work.
                  tf_buffer.setTransform(transform, "unused_authority",
                                         msg.getTopic() == kTfStaticTopic);
                } catch (const tf2::TransformException& ex) {
                  LOG(WARNING) << ex.what();
                }
              }
            }
            return false;
          } else {
            return true;
          }
        }));
  }

  while (playable_bag_multiplexer.IsMessageAvailable()) {
    const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
    const rosbag::MessageInstance& msg = std::get<0>(next_msg_tuple);
    const int trajectory_id = std::get<1>(next_msg_tuple);
    const bool bag_has_more_messages = std::get<2>(next_msg_tuple);

    if (!::ros::ok()) {
      return;
    }
    const std::string topic =
        node.node_handle()->resolveName(msg.getTopic(), false /* resolve */);
    if (expected_sensor_topics.at(trajectory_id).count(topic) == 0) {
      continue;
    }

    if (msg.isType<sensor_msgs::LaserScan>()) {
      node.HandleLaserScanMessage(trajectory_id, topic,
                                  msg.instantiate<sensor_msgs::LaserScan>());
    }
    if (msg.isType<sensor_msgs::MultiEchoLaserScan>()) {
      node.HandleMultiEchoLaserScanMessage(
          trajectory_id, topic,
          msg.instantiate<sensor_msgs::MultiEchoLaserScan>());
    }
    if (msg.isType<sensor_msgs::PointCloud2>()) {
      node.HandlePointCloud2Message(
          trajectory_id, topic, msg.instantiate<sensor_msgs::PointCloud2>());
    }
    if (msg.isType<sensor_msgs::Imu>()) {
      node.HandleImuMessage(trajectory_id, topic,
                            msg.instantiate<sensor_msgs::Imu>());
    }
    if (msg.isType<nav_msgs::Odometry>()) {
      node.HandleOdometryMessage(trajectory_id, topic,
                                 msg.instantiate<nav_msgs::Odometry>());
    }
    clock.clock = msg.getTime();
    clock_publisher.publish(clock);

    if (!bag_has_more_messages) {
      node.FinishTrajectory(trajectory_id);
    }
  }

  // Ensure the clock is republished after the bag has been finished, during the
  // final optimization, serialization, and optional indefinite spinning at the
  // end.
  clock_republish_timer.start();
  node.RunFinalOptimization();

  const std::chrono::time_point<std::chrono::steady_clock> end_time =
      std::chrono::steady_clock::now();
  const double wall_clock_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                                start_time)
          .count();

  LOG(INFO) << "Elapsed wall clock time: " << wall_clock_seconds << " s";
#ifdef __linux__
  timespec cpu_timespec = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec);
  LOG(INFO) << "Elapsed CPU time: "
            << (cpu_timespec.tv_sec + 1e-9 * cpu_timespec.tv_nsec) << " s";
  rusage usage;
  CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0) << strerror(errno);
  LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
#endif

  if (::ros::ok()) {
    const std::string output_filename = bag_filenames.front() + ".pbstream";
    LOG(INFO) << "Writing state to '" << output_filename << "'...";
    node.SerializeState(output_filename);
  }
  if (FLAGS_keep_running) {
    ::ros::waitForShutdown();
  }
}

}  // namespace cartographer_ros
