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

#include "cartographer_ros/sensor_bridge.h"

#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const string& CheckNoLeadingSlash(const string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/');
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const TfBridge* const tf_bridge,
    carto::mapping::TrajectoryBuilder* const trajectory_builder)
    : tf_bridge_(tf_bridge), trajectory_builder_(trajectory_builder) {}

void SensorBridge::HandleOdometryMessage(
    const string& topic, const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddOdometerData(
        topic, time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse());
  }
}

void SensorBridge::HandleImuMessage(const string& topic,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1);
  CHECK_NE(msg->angular_velocity_covariance[0], -1);
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking != nullptr) {
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame. "
           "Transforming linear acceleration into the tracking frame will "
           "otherwise be imprecise.";
    trajectory_builder_->AddImuData(
        topic, time,
        sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
        sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity));
  }
}

void SensorBridge::HandleLaserScanMessage(
    const string& topic, const sensor_msgs::LaserScan::ConstPtr& msg) {
  HandleRangefinder(topic, FromRos(msg->header.stamp), msg->header.frame_id,
                    carto::sensor::ToPointCloud(ToCartographer(*msg)));
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const string& topic, const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  HandleRangefinder(topic, FromRos(msg->header.stamp), msg->header.frame_id,
                    carto::sensor::ToPointCloud(ToCartographer(*msg)));
}

void SensorBridge::HandlePointCloud2Message(
    const string& topic, const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
  pcl::fromROSMsg(*msg, pcl_point_cloud);
  carto::sensor::PointCloud point_cloud;
  for (const auto& point : pcl_point_cloud) {
    point_cloud.emplace_back(point.x, point.y, point.z);
  }
  HandleRangefinder(topic, FromRos(msg->header.stamp), msg->header.frame_id,
                    point_cloud);
}

void SensorBridge::HandleRangefinder(const string& topic,
                                     const carto::common::Time time,
                                     const string& frame_id,
                                     const carto::sensor::PointCloud& ranges) {
  const auto sensor_to_tracking =
      tf_bridge_->LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddRangefinderData(
        topic, time, sensor_to_tracking->translation().cast<float>(),
        carto::sensor::TransformPointCloud(ranges,
                                           sensor_to_tracking->cast<float>()));
  }
}

}  // namespace cartographer_ros
