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
#include "cartographer_ros/urdf_reader.h"
#include "ros/callback_queue.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosgraph_msgs/Clock.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "urdf/model.h"

DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish the transforms from the bag.");
DEFINE_string(pbstream_filename, "",
              "If non-empty, filename of a pbstream to load.");
DEFINE_bool(keep_running, false,
            "Keep running the offline node after all messages from the bag "
            "have been processed.");

namespace cartographer_ros {

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;

void RunOfflineNode(
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    const cartographer_ros::NodeOptions& node_options,
    const cartographer_ros::TrajectoryOptions& trajectory_options,
    const std::vector<std::string>& bag_filenames) {
  const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();

  tf2_ros::Buffer tf_buffer;

  std::vector<geometry_msgs::TransformStamped> urdf_transforms;
  if (!FLAGS_urdf_filename.empty()) {
    urdf_transforms =
        ReadStaticTransformsFromUrdf(FLAGS_urdf_filename, &tf_buffer);
  }

  tf_buffer.setUsingDedicatedThread(true);

  Node node(node_options, std::move(map_builder), &tf_buffer);
  if (!FLAGS_pbstream_filename.empty()) {
    // TODO(jihoonl): LoadMap should be replaced by some better deserialization
    // of full SLAM state as non-frozen trajectories once possible
    node.LoadMap(FLAGS_pbstream_filename);
  }

  std::unordered_set<std::string> expected_sensor_ids;
  for (const std::string& topic :
       node.ComputeDefaultTopics(trajectory_options)) {
    CHECK(expected_sensor_ids.insert(node.node_handle()->resolveName(topic))
              .second);
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

  for (const std::string& bag_filename : bag_filenames) {
    if (!::ros::ok()) {
      break;
    }

    const int trajectory_id =
        node.AddOfflineTrajectory(expected_sensor_ids, trajectory_options);

    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);
    rosbag::View view(bag);
    const ::ros::Time begin_time = view.getBeginTime();
    const double duration_in_seconds = (view.getEndTime() - begin_time).toSec();

    // We need to keep 'tf_buffer' small because it becomes very inefficient
    // otherwise. We make sure that tf_messages are published before any data
    // messages, so that tf lookups always work.
    std::deque<rosbag::MessageInstance> delayed_messages;
    // We publish tf messages one second earlier than other messages. Under
    // the assumption of higher frequency tf this should ensure that tf can
    // always interpolate.
    const ::ros::Duration kDelay(1.);
    for (const rosbag::MessageInstance& msg : view) {
      if (!::ros::ok()) {
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
             delayed_messages.front().getTime() < msg.getTime() - kDelay) {
        const rosbag::MessageInstance& delayed_msg = delayed_messages.front();
        const std::string topic = node.node_handle()->resolveName(
            delayed_msg.getTopic(), false /* resolve */);
        if (delayed_msg.isType<sensor_msgs::LaserScan>()) {
          node.HandleLaserScanMessage(
              trajectory_id, topic,
              delayed_msg.instantiate<sensor_msgs::LaserScan>());
        }
        if (delayed_msg.isType<sensor_msgs::MultiEchoLaserScan>()) {
          node.HandleMultiEchoLaserScanMessage(
              trajectory_id, topic,
              delayed_msg.instantiate<sensor_msgs::MultiEchoLaserScan>());
        }
        if (delayed_msg.isType<sensor_msgs::PointCloud2>()) {
          node.HandlePointCloud2Message(
              trajectory_id, topic,
              delayed_msg.instantiate<sensor_msgs::PointCloud2>());
        }
        if (delayed_msg.isType<sensor_msgs::Imu>()) {
          node.HandleImuMessage(trajectory_id, topic,
                                delayed_msg.instantiate<sensor_msgs::Imu>());
        }
        if (delayed_msg.isType<nav_msgs::Odometry>()) {
          node.HandleOdometryMessage(
              trajectory_id, topic,
              delayed_msg.instantiate<nav_msgs::Odometry>());
        }
        if (delayed_msg.isType<sensor_msgs::NavSatFix>()) {
          node.HandleNavSatFixMessage(
              trajectory_id, topic,
              delayed_msg.instantiate<sensor_msgs::NavSatFix>());
        }
        clock.clock = delayed_msg.getTime();
        clock_publisher.publish(clock);

        LOG_EVERY_N(INFO, 100000)
            << "Processed " << (delayed_msg.getTime() - begin_time).toSec()
            << " of " << duration_in_seconds << " bag time seconds...";

        delayed_messages.pop_front();
      }

      const std::string topic =
          node.node_handle()->resolveName(msg.getTopic(), false /* resolve */);
      if (expected_sensor_ids.count(topic) == 0) {
        continue;
      }
      delayed_messages.push_back(msg);
    }

    bag.close();
    node.FinishTrajectory(trajectory_id);
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
