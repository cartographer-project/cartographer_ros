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

#include "cartographer/mapping/sensor_collator.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_data.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

// A wrapper around SensorCollator that converts ROS messages into our internal
// representation and passes them on to the 'sensor_collator'.
class SensorBridge {
 public:
  explicit SensorBridge(
      int trajectory_id,
      ::cartographer::mapping::SensorCollator<SensorData>* sensor_collator);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

  void AddOdometryMessage(const string& topic,
                          const nav_msgs::Odometry::ConstPtr& msg);
  void AddImuMessage(const string& topic,
                     const sensor_msgs::Imu::ConstPtr& msg);
  void AddLaserScanMessage(const string& topic,
                           const sensor_msgs::LaserScan::ConstPtr& msg);
  void AddMultiEchoLaserScanMessage(
      const string& topic,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void AddPointCloud2Message(const string& topic,
                             const sensor_msgs::PointCloud2::ConstPtr& msg);

 private:
  const int trajectory_id_;
  ::cartographer::mapping::SensorCollator<SensorData>* sensor_collator_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
