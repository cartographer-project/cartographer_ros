/*
 * Copyright 2017 The Cartographer Authors
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

#include <iostream>
#include <map>
#include <string>

#include "cartographer/common/histogram.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/buffer.h"
#include "urdf/model.h"

DEFINE_string(bag_filename, "", "Bag to process.");

namespace cartographer_ros {
namespace {

struct TimestampData {
  ros::Time last_timestamp;
  string topic;
  ::cartographer::common::Histogram histogram;
};

void Run(const string& bag_filename) {
  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag);
  const ::ros::Time begin_time = view.getBeginTime();

  std::map<string, TimestampData> timestamp_data;
  for (const rosbag::MessageInstance& message : view) {
    string frame_id;
    ros::Time time;
    if (message.isType<sensor_msgs::PointCloud2>()) {
      auto msg = message.instantiate<sensor_msgs::PointCloud2>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
    } else if (message.isType<sensor_msgs::MultiEchoLaserScan>()) {
      auto msg = message.instantiate<sensor_msgs::MultiEchoLaserScan>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
    } else if (message.isType<sensor_msgs::LaserScan>()) {
      auto msg = message.instantiate<sensor_msgs::LaserScan>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
    } else if (message.isType<sensor_msgs::Imu>()) {
      auto msg = message.instantiate<sensor_msgs::Imu>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
    } else if (message.isType<nav_msgs::Odometry>()) {
      auto msg = message.instantiate<nav_msgs::Odometry>();
      time = msg->header.stamp;
      frame_id = msg->header.frame_id;
    } else {
      continue;
    }

    if (!timestamp_data.count(frame_id)) {
      timestamp_data.emplace(
          frame_id, TimestampData{time, message.getTopic(),
                                  ::cartographer::common::Histogram()});
    } else {
      auto& entry = timestamp_data.at(frame_id);
      if (entry.topic != message.getTopic()) {
        LOG(ERROR) << "frame_id \"" << frame_id
                   << "\" is send on multiple topics. It was seen at least on "
                   << entry.topic << " and " << message.getTopic();
      }
      const double delta_t_sec = (time - entry.last_timestamp).toSec();
      if (delta_t_sec < 0) {
        LOG(ERROR) << "Sensor with frame_id \"" << frame_id
                   << "\" jumps backwards in time. Make sure that the bag "
                      "contains the data for each frame_id sorted by "
                      "header.stamp, i.e. the order in which they were "
                      "acquired from the sensor.";
      }
      entry.histogram.Add(delta_t_sec);
      entry.last_timestamp = time;
    }
  }
  bag.close();

  constexpr int kNumBucketsForHistogram = 10;
  for (const auto& entry_pair : timestamp_data) {
    LOG(INFO) << "Time delta histogram for consecutive messages on topic \""
              << entry_pair.second.topic << "\" (frame_id: \""
              << entry_pair.first << "\"):\n"
              << entry_pair.second.histogram.ToString(kNumBucketsForHistogram);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  ::cartographer_ros::Run(FLAGS_bag_filename);
}
