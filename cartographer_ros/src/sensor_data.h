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

namespace cartographer_ros {

// This type is a logical union, i.e. only one proto is actually filled in. It
// is only used for time ordering sensor data before passing it on.
enum class SensorType { kImu, kLaserScan, kLaserFan3D, kOdometry };
struct SensorData {
  struct Odometry {
    ::cartographer::transform::Rigid3d pose;
    ::cartographer::kalman_filter::PoseCovariance covariance;
  };

  SensorData(const string& frame_id, ::cartographer::sensor::proto::Imu imu);
  SensorData(const string& frame_id,
             ::cartographer::sensor::proto::LaserScan laser_scan);
  SensorData(const string& frame_id,
             ::cartographer::sensor::proto::LaserFan3D laser_fan_3d);
  SensorData(const string& frame_id, const Odometry& odometry);

  SensorType type;
  string frame_id;
  ::cartographer::sensor::proto::Imu imu;
  ::cartographer::sensor::proto::LaserScan laser_scan;
  ::cartographer::sensor::proto::LaserFan3D laser_fan_3d;
  Odometry odometry;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_SENSOR_DATA_H_
