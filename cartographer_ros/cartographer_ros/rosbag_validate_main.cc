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

#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "cartographer/common/histogram.h"
#include "cartographer/common/make_unique.h"
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
DEFINE_bool(dump_timing, false,
            "Dump per-sensor timing information in files called "
            "timing_<frame_id>.csv in the current directory.");

namespace cartographer_ros {
namespace {

struct PerFrameId {
  ros::Time last_timestamp;
  std::string topic;
  ::cartographer::common::Histogram histogram;
  std::unique_ptr<std::ofstream> timing_file;
};

std::unique_ptr<std::ofstream> CreateTimingFile(const std::string& frame_id) {
  auto timing_file = ::cartographer::common::make_unique<std::ofstream>(
      std::string("timing_") + frame_id + ".csv", std::ios_base::out);

  (*timing_file)
      << "# Timing information for sensor with frame id: " << frame_id
      << std::endl
      << "# Columns are in order" << std::endl
      << "# - packet index of the packet in the bag, first packet is 1"
      << std::endl
      << "# - timestamp when rosbag wrote the packet, i.e. "
         "rosbag::MessageInstance::getTime().toNSec()"
      << std::endl
      << "# - timestamp when data was acquired, i.e. "
         "message.header.stamp.toNSec()"
      << std::endl
      << "#" << std::endl
      << "# The data can be read in python using" << std::endl
      << "# import numpy" << std::endl
      << "# np.loadtxt(<filename>, dtype='uint64')" << std::endl;

  return timing_file;
}

void Run(const std::string& bag_filename, const bool dump_timing) {
  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag);

  std::map<std::string, PerFrameId> per_frame_id;
  size_t message_index = 0;
  for (const rosbag::MessageInstance& message : view) {
    ++message_index;
    std::string frame_id;
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

    bool first_packet = false;
    if (!per_frame_id.count(frame_id)) {
      per_frame_id.emplace(
          frame_id,
          PerFrameId{time, message.getTopic(),
                     ::cartographer::common::Histogram(),
                     dump_timing ? CreateTimingFile(frame_id) : nullptr});
      first_packet = true;
    }

    auto& entry = per_frame_id.at(frame_id);
    if (!first_packet) {
      const double delta_t_sec = (time - entry.last_timestamp).toSec();
      if (delta_t_sec < 0) {
        LOG(ERROR) << "Sensor with frame_id \"" << frame_id
                   << "\" jumps backwards in time. Make sure that the bag "
                      "contains the data for each frame_id sorted by "
                      "header.stamp, i.e. the order in which they were "
                      "acquired from the sensor.";
      }
      entry.histogram.Add(delta_t_sec);
    }

    if (entry.topic != message.getTopic()) {
      LOG(ERROR) << "frame_id \"" << frame_id
                 << "\" is send on multiple topics. It was seen at least on "
                 << entry.topic << " and " << message.getTopic();
    }
    entry.last_timestamp = time;

    if (dump_timing) {
      CHECK(entry.timing_file != nullptr);
      (*entry.timing_file) << message_index << "\t"
                           << message.getTime().toNSec() << "\t"
                           << time.toNSec() << std::endl;
    }
  }
  bag.close();

  constexpr int kNumBucketsForHistogram = 10;
  for (const auto& entry_pair : per_frame_id) {
    LOG(INFO) << "Time delta histogram for consecutive messages on topic \""
              << entry_pair.second.topic << "\" (frame_id: \""
              << entry_pair.first << "\"):\n"
              << entry_pair.second.histogram.ToString(kNumBucketsForHistogram);
  }

  if (dump_timing) {
    for (const auto& entry_pair : per_frame_id) {
      entry_pair.second.timing_file->close();
      CHECK(*entry_pair.second.timing_file)
          << "Could not write timing information for \"" << entry_pair.first
          << "\"";
    }
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_bag_filename.empty()) << "-bag_filename is missing.";
  ::cartographer_ros::Run(FLAGS_bag_filename, FLAGS_dump_timing);
}
