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
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/urdf_reader.h"
#include "ros/callback_queue.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosgraph_msgs/Clock.h"
#include "urdf/model.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
// TODO(damonkohler): Support multi-trajectory.
DEFINE_string(bag_filename, "", "Bag to process.");
DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");

namespace cartographer_ros {
namespace {

constexpr int kLatestOnlyPublisherQueueSize = 1;
constexpr char kClockTopic[] = "clock";

void Run() {
  // TODO(damonkohler): Pull out this common code across binaries.
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  tf2_ros::Buffer tf_buffer;
  tf_buffer.setUsingDedicatedThread(true);
  ReadStaticTransformsFromUrdf(FLAGS_urdf_filename, &tf_buffer);
  const auto options = CreateNodeOptions(&lua_parameter_dictionary);
  Node node(options, &tf_buffer);
  node.Initialize();

  std::unordered_set<string> expected_sensor_ids;

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  if (options.use_laser_scan) {
    expected_sensor_ids.insert(
        node.node_handle()->resolveName(kLaserScanTopic, false /* remap */));
  }
  if (options.use_multi_echo_laser_scan) {
    expected_sensor_ids.insert(node.node_handle()->resolveName(
        kMultiEchoLaserScanTopic, false /* remap */));
  }

  // For 3D SLAM, subscribe to all point clouds topics.
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = kPointCloud2Topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      expected_sensor_ids.insert(
          node.node_handle()->resolveName(topic, false /* remap */));
    }
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (options.map_builder_options.use_trajectory_builder_3d() ||
      (options.map_builder_options.use_trajectory_builder_2d() &&
       options.map_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_sensor_ids.insert(
        node.node_handle()->resolveName(kImuTopic, false /* remap */));
  }

  // For both 2D and 3D SLAM, odometry is optional.
  if (options.use_odometry) {
    expected_sensor_ids.insert(
        node.node_handle()->resolveName(kOdometryTopic, false /* remap */));
  }

  // TODO(damonkohler): Support multi-trajectory.
  const int trajectory_id = node.map_builder_bridge()->AddTrajectory(
      expected_sensor_ids, options.tracking_frame);

  ::ros::Publisher clock_publisher =
      node.node_handle()->advertise<rosgraph_msgs::Clock>(
          kClockTopic, kLatestOnlyPublisherQueueSize);

  rosbag::Bag bag;
  bag.open(FLAGS_bag_filename, rosbag::bagmode::Read);
  for (const rosbag::MessageInstance& msg : rosbag::View(bag)) {
    if (!::ros::ok()) {
      break;
    }

    const string& topic = node.node_handle()->resolveName(msg.getTopic());
    if (expected_sensor_ids.count(
            node.node_handle()->resolveName(msg.getTopic())) == 0) {
      continue;
    }
    if (msg.isType<sensor_msgs::LaserScan>()) {
      node.map_builder_bridge()
          ->sensor_bridge(trajectory_id)
          ->HandleLaserScanMessage(topic,
                                   msg.instantiate<sensor_msgs::LaserScan>());
    } else if (msg.isType<sensor_msgs::MultiEchoLaserScan>()) {
      node.map_builder_bridge()
          ->sensor_bridge(trajectory_id)
          ->HandleMultiEchoLaserScanMessage(
              topic, msg.instantiate<sensor_msgs::MultiEchoLaserScan>());
    } else if (msg.isType<sensor_msgs::PointCloud2>()) {
      node.map_builder_bridge()
          ->sensor_bridge(trajectory_id)
          ->HandlePointCloud2Message(
              topic, msg.instantiate<sensor_msgs::PointCloud2>());
    } else if (msg.isType<sensor_msgs::Imu>()) {
      node.map_builder_bridge()
          ->sensor_bridge(trajectory_id)
          ->HandleImuMessage(topic, msg.instantiate<sensor_msgs::Imu>());
    } else if (msg.isType<nav_msgs::Odometry>()) {
      node.map_builder_bridge()
          ->sensor_bridge(trajectory_id)
          ->HandleOdometryMessage(topic, msg.instantiate<nav_msgs::Odometry>());
    }

    rosgraph_msgs::Clock clock;
    clock.clock = msg.getTime();
    clock_publisher.publish(clock);

    ::ros::spinOnce();
  }
  bag.close();

  node.map_builder_bridge()->FinishTrajectory(trajectory_id);
  node.map_builder_bridge()->WriteAssets(FLAGS_bag_filename);
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
  CHECK(!FLAGS_urdf_filename.empty()) << "-urdf_filename is missing.";

  ::ros::init(argc, argv, "cartographer_offline_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();

  ::ros::shutdown();
}
