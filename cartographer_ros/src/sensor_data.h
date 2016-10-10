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

#ifndef CARTOGRAPHER_ROS_SENSOR_DATA_H_
#define CARTOGRAPHER_ROS_SENSOR_DATA_H_

#include <string>

#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer_ros {

// This type is a logical union, i.e. only one type of sensor data is actually
// filled in. It is only used for time ordering sensor data before passing it
// on.
enum class SensorType { kImu, kLaserScan, kLaserFan3D, kOdometry };
struct SensorData {
  struct Odometry {
    ::cartographer::transform::Rigid3d pose;
    ::cartographer::kalman_filter::PoseCovariance covariance;
  };

  struct Imu {
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d linear_acceleration;
  };

  SensorData(const string& frame_id, const Imu& imu)
      : type(SensorType::kImu),
        frame_id(CheckNoLeadingSlash(frame_id)),
        imu(imu) {}

  SensorData(const string& frame_id, const ::cartographer::sensor::proto::LaserScan& laser_scan)
      : type(SensorType::kLaserScan),
        frame_id(CheckNoLeadingSlash(frame_id)),
        laser_scan(laser_scan) {}

  SensorData(const string& frame_id, const ::cartographer::sensor::proto::LaserFan3D& laser_fan_3d)
      : type(SensorType::kLaserFan3D),
        frame_id(CheckNoLeadingSlash(frame_id)),
        laser_fan_3d(laser_fan_3d) {}

  SensorData(const string& frame_id, const Odometry& odometry)
      : type(SensorType::kOdometry), frame_id(frame_id), odometry(odometry) {}

  SensorType type;
  string frame_id;
  Imu imu;
  ::cartographer::sensor::proto::LaserScan laser_scan;
  ::cartographer::sensor::proto::LaserFan3D laser_fan_3d;
  Odometry odometry;

  private:
    static const string& CheckNoLeadingSlash(const string& frame_id) {
      if (frame_id.size() > 0) {
        CHECK_NE(frame_id[0], '/');
      }
      return frame_id;
    }
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_SENSOR_DATA_H_
