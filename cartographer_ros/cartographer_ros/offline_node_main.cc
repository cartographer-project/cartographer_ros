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

#include <errno.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/split_string.h"
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
DEFINE_string(pbstream_filename, "",
              "If non-empty, filename of a pbstream to load.");
DEFINE_bool(keep_running, false,
            "Keep running the offline node after all messages from the bag "
            "have been processed.");

namespace cartographer_ros {
namespace {

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
constexpr int kSingleThreaded = 1;

// A poor mans typed union that signifies events from the IO thread.
struct Event {
  enum class Kind {
    kNewBag,
    kBagEnd,
    kRosMessage,
  };

  // A poors mans union that contains instantiated ROS messages.
  struct RosMessage {
    enum class Kind {
      kTf,
      kImu,
      kOdometry,
      kLaserScan,
      kMultiEchoLaserScan,
      kPointCloud2,
    };

    RosMessage::Kind kind;
    string topic;
    ros::Time serialization_time;
    sensor_msgs::Imu::ConstPtr imu;
    sensor_msgs::LaserScan::ConstPtr laser_scan;
    sensor_msgs::MultiEchoLaserScan::ConstPtr multi_echo_laser_scan;
    sensor_msgs::PointCloud2::ConstPtr point_cloud2;
    nav_msgs::Odometry::ConstPtr odometry;
    tf2_msgs::TFMessage::ConstPtr tf;
  };

  static Event NewBag(::ros::Time begin_time, double duration_in_seconds) {
    Event event;
    event.kind = Kind::kNewBag;
    event.new_bag.begin_time = begin_time;
    event.new_bag.duration_in_seconds = duration_in_seconds;
    return event;
  }

  static Event BagEnd() {
    Event event;
    event.kind = Kind::kBagEnd;
    return event;
  }

  static Event TfMessage(const string& topic,
                         const ros::Time serialization_time,
                         tf2_msgs::TFMessage::ConstPtr ros_message) {
    CHECK(ros_message != nullptr);
    Event event;
    event.kind = Kind::kRosMessage;
    event.message.kind = RosMessage::Kind::kTf;
    event.message.topic = topic;
    event.message.serialization_time = serialization_time;
    event.message.tf = std::move(ros_message);
    return event;
  }

  static Event LaserScanMessage(const string& topic,
                                const ros::Time serialization_time,
                                sensor_msgs::LaserScan::ConstPtr ros_message) {
    CHECK(ros_message != nullptr);
    Event event;
    event.kind = Kind::kRosMessage;
    event.message.kind = RosMessage::Kind::kLaserScan;
    event.message.topic = topic;
    event.message.serialization_time = serialization_time;
    event.message.laser_scan = std::move(ros_message);
    return event;
  }

  static Event MultiEchoLaserScanMessage(
      const string& topic, const ros::Time serialization_time,
      sensor_msgs::MultiEchoLaserScan::ConstPtr ros_message) {
    CHECK(ros_message != nullptr);
    Event event;
    event.kind = Kind::kRosMessage;
    event.message.kind = RosMessage::Kind::kMultiEchoLaserScan;
    event.message.topic = topic;
    event.message.serialization_time = serialization_time;
    event.message.multi_echo_laser_scan = std::move(ros_message);
    return event;
  }

  static Event PointCloud2Message(
      const string& topic, const ros::Time serialization_time,
      sensor_msgs::PointCloud2::ConstPtr ros_message) {
    CHECK(ros_message != nullptr);
    Event event;
    event.kind = Kind::kRosMessage;
    event.message.kind = RosMessage::Kind::kPointCloud2;
    event.message.topic = topic;
    event.message.serialization_time = serialization_time;
    event.message.point_cloud2 = std::move(ros_message);
    return event;
  }

  static Event ImuMessage(const string& topic,
                          const ros::Time serialization_time,
                          sensor_msgs::Imu::ConstPtr ros_message) {
    CHECK(ros_message != nullptr);
    Event event;
    event.kind = Kind::kRosMessage;
    event.message.kind = RosMessage::Kind::kImu;
    event.message.topic = topic;
    event.message.serialization_time = serialization_time;
    event.message.imu = std::move(ros_message);
    return event;
  }

  static Event OdometryMessage(const string& topic,
                               const ros::Time serialization_time,
                               nav_msgs::Odometry::ConstPtr ros_message) {
    CHECK(ros_message != nullptr);
    Event event;
    event.kind = Kind::kRosMessage;
    event.message.kind = RosMessage::Kind::kOdometry;
    event.message.topic = topic;
    event.message.serialization_time = serialization_time;
    event.message.odometry = std::move(ros_message);
    return event;
  }

  Kind kind;

  struct {
    ::ros::Time begin_time;
    double duration_in_seconds;
  } new_bag;
  RosMessage message;
};

void RosbagIoThread(const std::vector<string>& bag_filenames,
                    const std::unordered_set<string>& expected_sensor_ids,
                    ::ros::NodeHandle* node_handle,
                    ::cartographer::common::BlockingQueue<Event>* queue) {
  for (size_t bag_index = 0; bag_index < bag_filenames.size(); ++bag_index) {
    const string& bag_filename = bag_filenames[bag_index];
    if (!::ros::ok()) {
      break;
    }

    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);
    rosbag::View view(bag);
    const ::ros::Time begin_time = view.getBeginTime();
    queue->Push(
        Event::NewBag(begin_time, (view.getEndTime() - begin_time).toSec()));

    for (const rosbag::MessageInstance& msg : view) {
      if (!::ros::ok()) {
        break;
      }
      const string topic =
          node_handle->resolveName(msg.getTopic(), false /* resolve */);

      if (msg.isType<tf2_msgs::TFMessage>()) {
        queue->Push(Event::TfMessage(topic, msg.getTime(),
                                     msg.instantiate<tf2_msgs::TFMessage>()));
        continue;
      }

      if (expected_sensor_ids.count(topic) == 0) {
        continue;
      }

      if (msg.isType<sensor_msgs::LaserScan>()) {
        queue->Push(Event::LaserScanMessage(
            topic, msg.getTime(), msg.instantiate<sensor_msgs::LaserScan>()));
      } else if (msg.isType<sensor_msgs::MultiEchoLaserScan>()) {
        queue->Push(Event::MultiEchoLaserScanMessage(
            topic, msg.getTime(),
            msg.instantiate<sensor_msgs::MultiEchoLaserScan>()));
      } else if (msg.isType<sensor_msgs::PointCloud2>()) {
        queue->Push(Event::PointCloud2Message(
            topic, msg.getTime(), msg.instantiate<sensor_msgs::PointCloud2>()));
      } else if (msg.isType<sensor_msgs::Imu>()) {
        queue->Push(Event::ImuMessage(topic, msg.getTime(),
                                      msg.instantiate<sensor_msgs::Imu>()));
      } else if (msg.isType<nav_msgs::Odometry>()) {
        queue->Push(Event::OdometryMessage(
            topic, msg.getTime(), msg.instantiate<nav_msgs::Odometry>()));
      }
    }
    queue->Push(Event::BagEnd());
    bag.close();
  }
}

void Run(const std::vector<string>& bag_filenames) {
  const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

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

  ::cartographer::common::BlockingQueue<Event> queue(100);
  std::thread rosbag_io_thread(RosbagIoThread, std::ref(bag_filenames),
                               std::ref(expected_sensor_ids),
                               node.node_handle(), &queue);

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

  ::ros::Time begin_time;
  double duration_in_seconds;
  int trajectory_id = -1;

  // We need to keep 'tf_buffer' small because it becomes very inefficient
  // otherwise. We make sure that tf_messages are published before any data
  // messages, so that tf lookups always work.
  std::deque<Event::RosMessage> delayed_messages;

  // We publish tf messages one second earlier than other messages. Under
  // the assumption of higher frequency tf this should ensure that tf can
  // always interpolate.
  const ::ros::Duration kDelay(1.);

  for (size_t bag_index = 0; bag_index < bag_filenames.size();) {
    const Event message = queue.Pop();
    switch (message.kind) {
      case Event::Kind::kNewBag:
        begin_time = message.new_bag.begin_time;
        duration_in_seconds = message.new_bag.duration_in_seconds;
        trajectory_id =
            node.AddOfflineTrajectory(expected_sensor_ids, trajectory_options);
        break;

      case Event::Kind::kBagEnd:
        node.FinishTrajectory(trajectory_id);
        ++bag_index;
        break;

      case Event::Kind::kRosMessage: {
        if (FLAGS_use_bag_transforms &&
            message.message.kind == Event::RosMessage::Kind::kTf) {
          tf_publisher.publish(*message.message.tf);
          for (const auto& transform : message.message.tf->transforms) {
            try {
              tf_buffer.setTransform(transform, "unused_authority",
                                     message.message.topic == kTfStaticTopic);
            } catch (const tf2::TransformException& ex) {
              LOG(WARNING) << ex.what();
            }
          }
          // NOCOM(#hrapp): ugly.
          break;  // Alread
        }

        while (!delayed_messages.empty() &&
               delayed_messages.front().serialization_time <
                   message.message.serialization_time - kDelay) {
          const Event::RosMessage& delayed_msg = delayed_messages.front();
          CHECK_NE(trajectory_id, -1)
              << "Did not see a kNewBag message before first ROS message.";
          switch (delayed_msg.kind) {
            case Event::RosMessage::Kind::kLaserScan:
              node.HandleLaserScanMessage(trajectory_id, delayed_msg.topic,
                                          delayed_msg.laser_scan);
              break;

            case Event::RosMessage::Kind::kMultiEchoLaserScan:
              node.HandleMultiEchoLaserScanMessage(
                  trajectory_id, delayed_msg.topic,
                  delayed_msg.multi_echo_laser_scan);
              break;

            case Event::RosMessage::Kind::kPointCloud2:
              node.HandlePointCloud2Message(trajectory_id, delayed_msg.topic,
                                            delayed_msg.point_cloud2);
              break;

            case Event::RosMessage::Kind::kImu:
              node.HandleImuMessage(trajectory_id, delayed_msg.topic,
                                    delayed_msg.imu);
              break;

            case Event::RosMessage::Kind::kOdometry:
              node.HandleOdometryMessage(trajectory_id, delayed_msg.topic,
                                         delayed_msg.odometry);
              break;

            case Event::RosMessage::Kind::kTf:
              LOG(FATAL) << "Should already been handled.";
              break;
          }

          clock.clock = delayed_msg.serialization_time;
          clock_publisher.publish(clock);

          LOG_EVERY_N(INFO, 100000)
              << "Processed "
              << (delayed_msg.serialization_time - begin_time).toSec() << " of "
              << duration_in_seconds << " bag time seconds...";
          delayed_messages.pop_front();
        }
        delayed_messages.push_back(message.message);
        break;
      }
    }
  }

  rosbag_io_thread.join();

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

  ::ros::init(argc, argv, "cartographer_offline_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run(
      cartographer_ros::SplitString(FLAGS_bag_filenames, ','));

  ::ros::shutdown();
}
