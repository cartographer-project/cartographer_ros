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

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

constexpr int kInfiniteSubscriberQueueSize = 0;

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";

void Run() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  const auto options = CreateNodeOptions(&lua_parameter_dictionary);
  Node node(options);

  int trajectory_id = -1;
  std::unordered_set<string> expected_sensor_ids;

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  ::ros::Subscriber laser_scan_subscriber;
  if (options.use_laser_scan) {
    laser_scan_subscriber = node.node_handle()->subscribe(
        kLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
            [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
              node.map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleLaserScanMessage(kLaserScanTopic, msg);
            }));
    expected_sensor_ids.insert(kLaserScanTopic);
  }
  if (options.use_multi_echo_laser_scan) {
    laser_scan_subscriber = node.node_handle()->subscribe(
        kMultiEchoLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::MultiEchoLaserScan::ConstPtr&)>(
            [&](const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
              node.map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleMultiEchoLaserScanMessage(kMultiEchoLaserScanTopic,
                                                    msg);
            }));
    expected_sensor_ids.insert(kMultiEchoLaserScanTopic);
  }

  // For 3D SLAM, subscribe to all point clouds topics.
  std::vector<::ros::Subscriber> point_cloud_subscribers;
  if (options.num_point_clouds > 0) {
    for (int i = 0; i < options.num_point_clouds; ++i) {
      string topic = kPointCloud2Topic;
      if (options.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      point_cloud_subscribers.push_back(node.node_handle()->subscribe(
          topic, kInfiniteSubscriberQueueSize,
          boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
              [&, topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                node.map_builder_bridge()
                    ->sensor_bridge(trajectory_id)
                    ->HandlePointCloud2Message(topic, msg);
              })));
      expected_sensor_ids.insert(topic);
    }
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  ::ros::Subscriber imu_subscriber;
  if (options.map_builder_options.use_trajectory_builder_3d() ||
      (options.map_builder_options.use_trajectory_builder_2d() &&
       options.map_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    imu_subscriber = node.node_handle()->subscribe(
        kImuTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::Imu::ConstPtr& msg)>(
            [&](const sensor_msgs::Imu::ConstPtr& msg) {
              node.map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleImuMessage(kImuTopic, msg);
            }));
    expected_sensor_ids.insert(kImuTopic);
  }

  // For both 2D and 3D SLAM, odometry is optional.
  ::ros::Subscriber odometry_subscriber;
  if (options.use_odometry) {
    odometry_subscriber = node.node_handle()->subscribe(
        kOdometryTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
            [&](const nav_msgs::Odometry::ConstPtr& msg) {
              node.map_builder_bridge()
                  ->sensor_bridge(trajectory_id)
                  ->HandleOdometryMessage(kOdometryTopic, msg);
            }));
    expected_sensor_ids.insert(kOdometryTopic);
  }

  trajectory_id = node.map_builder_bridge()->AddTrajectory(
      expected_sensor_ids, options.tracking_frame);

  ::ros::ServiceServer finish_trajectory_server =
      node.node_handle()->advertiseService(
          kFinishTrajectoryServiceName,
          boost::function<bool(
              ::cartographer_ros_msgs::FinishTrajectory::Request&,
              ::cartographer_ros_msgs::FinishTrajectory::Response&)>(
              [&](::cartographer_ros_msgs::FinishTrajectory::Request& request,
                  ::cartographer_ros_msgs::FinishTrajectory::Response&) {
                const int previous_trajectory_id = trajectory_id;
                trajectory_id = node.map_builder_bridge()->AddTrajectory(
                    expected_sensor_ids, options.tracking_frame);
                node.map_builder_bridge()->FinishTrajectory(
                    previous_trajectory_id);
                node.map_builder_bridge()->WriteAssets(request.stem);
                return true;
              }));

  node.Initialize();
  node.Spin();

  node.map_builder_bridge()->FinishTrajectory(trajectory_id);
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

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
