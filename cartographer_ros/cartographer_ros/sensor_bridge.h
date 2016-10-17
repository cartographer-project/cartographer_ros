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

#ifndef CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
#define CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_

#include "cartographer/sensor/collator.h"
#include "cartographer/sensor/data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/tf_bridge.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

struct SensorBridgeOptions {
  double horizontal_laser_min_range;
  double horizontal_laser_max_range;
  double horizontal_laser_missing_echo_ray_length;
  double constant_odometry_translational_variance;
  double constant_odometry_rotational_variance;
};

SensorBridgeOptions CreateSensorBridgeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge {
 public:
  explicit SensorBridge(
      const SensorBridgeOptions& options, const TfBridge* tf_bridge,
      int trajectory_id,
      ::cartographer::sensor::Collator<::cartographer::sensor::Data>*
          sensor_collator);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

  void HandleOdometryMessage(const string& topic,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleImuMessage(const string& topic,
                        const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(const string& topic,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      const string& topic,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(const string& topic,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

 private:
  void HandleLaserScanProto(
      const string& topic, const ::cartographer::common::Time time,
      const string& frame_id,
      const ::cartographer::sensor::proto::LaserScan& laser_scan);

  const SensorBridgeOptions options_;
  const TfBridge* const tf_bridge_;
  const int trajectory_id_;
  ::cartographer::sensor::Collator<::cartographer::sensor::Data>* const
      sensor_collator_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
