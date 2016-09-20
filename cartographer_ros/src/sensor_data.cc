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

#include "sensor_data.h"

#include <string>

#include "glog/logging.h"

namespace cartographer_ros {

namespace {

namespace proto = ::cartographer::sensor::proto;

using ::cartographer::transform::Rigid3d;
using ::cartographer::kalman_filter::PoseCovariance;

const string& CheckNoLeadingSlash(const string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/');
  }
  return frame_id;
}

}  // namespace

SensorData::SensorData(const string& frame_id, proto::Imu imu)
    : type(SensorType::kImu),
      frame_id(CheckNoLeadingSlash(frame_id)),
      imu(imu) {}

SensorData::SensorData(const string& frame_id, proto::LaserScan laser_scan)
    : type(SensorType::kLaserScan),
      frame_id(CheckNoLeadingSlash(frame_id)),
      laser_scan(laser_scan) {}

SensorData::SensorData(const string& frame_id, proto::LaserFan3D laser_fan_3d)
    : type(SensorType::kLaserFan3D),
      frame_id(CheckNoLeadingSlash(frame_id)),
      laser_fan_3d(laser_fan_3d) {}

SensorData::SensorData(const string& frame_id, const Odometry& odometry)
    : type(SensorType::kOdometry), frame_id(frame_id), odometry(odometry) {}

}  // namespace cartorapher_ros
