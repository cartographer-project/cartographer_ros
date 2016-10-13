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

SensorBridgeOptions CreateSensorBridgeOptions(
    carto::common::LuaParameterDictionary* const lua_parameter_dictionary) {
  SensorBridgeOptions options;
  options.horizontal_laser_min_range =
      lua_parameter_dictionary->GetDouble("horizontal_laser_min_range");
  options.horizontal_laser_max_range =
      lua_parameter_dictionary->GetDouble("horizontal_laser_max_range");
  options.horizontal_laser_missing_echo_ray_length =
      lua_parameter_dictionary->GetDouble(
          "horizontal_laser_missing_echo_ray_length");
  options.constant_odometry_translational_variance =
      lua_parameter_dictionary->GetDouble(
          "constant_odometry_translational_variance");
  options.constant_odometry_rotational_variance =
      lua_parameter_dictionary->GetDouble(
          "constant_odometry_rotational_variance");
  return options;
}

SensorBridge::SensorBridge(
    const SensorBridgeOptions& options, const TfBridge* const tf_bridge,
    const int trajectory_id,
    carto::mapping::SensorCollator<carto::sensor::Data>* const sensor_collator)
    : options_(options),
      tf_bridge_(tf_bridge),
      trajectory_id_(trajectory_id),
      sensor_collator_(sensor_collator) {}

void SensorBridge::HandleOdometryMessage(
    const string& topic, const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  const Eigen::Matrix3d translational =
      Eigen::Matrix3d::Identity() *
      options_.constant_odometry_translational_variance;
  const Eigen::Matrix3d rotational =
      Eigen::Matrix3d::Identity() *
      options_.constant_odometry_rotational_variance;
  // clang-format off
  carto::kalman_filter::PoseCovariance covariance;
  covariance <<
      translational, Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), rotational;
  // clang-format on
  const auto sensor_to_tracking =
      tf_bridge_->LookupToTracking(time, msg->child_frame_id);
  if (sensor_to_tracking != nullptr) {
    sensor_collator_->AddSensorData(
        trajectory_id_, carto::common::ToUniversal(time), topic,
        carto::common::make_unique<::cartographer::sensor::Data>(
            CheckNoLeadingSlash(msg->child_frame_id),
            ::cartographer::sensor::Data::Odometry{
                ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse(),
                covariance}));
  }
}

void SensorBridge::HandleImuMessage(const string& topic,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1);
  CHECK_NE(msg->angular_velocity_covariance[0], -1);
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking =
      tf_bridge_->LookupToTracking(time, msg->header.frame_id);
  if (sensor_to_tracking != nullptr) {
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame. "
           "Transforming linear acceleration into the tracking frame will "
           "otherwise be imprecise.";
    sensor_collator_->AddSensorData(
        trajectory_id_, carto::common::ToUniversal(time), topic,
        carto::common::make_unique<::cartographer::sensor::Data>(
            CheckNoLeadingSlash(msg->header.frame_id),
            ::cartographer::sensor::Data::Imu{
                sensor_to_tracking->rotation() *
                    ToEigen(msg->linear_acceleration),
                sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity),
            }));
  }
}

void SensorBridge::HandleLaserScanMessage(
    const string& topic, const sensor_msgs::LaserScan::ConstPtr& msg) {
  HandleLaserScanProto(topic, FromRos(msg->header.stamp), msg->header.frame_id,
                       ToCartographer(*msg));
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const string& topic, const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  HandleLaserScanProto(topic, FromRos(msg->header.stamp), msg->header.frame_id,
                       ToCartographer(*msg));
}

void SensorBridge::HandlePointCloud2Message(
    const string& topic, const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
  pcl::fromROSMsg(*msg, pcl_point_cloud);
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking =
      tf_bridge_->LookupToTracking(time, msg->header.frame_id);
  if (sensor_to_tracking != nullptr) {
    sensor_collator_->AddSensorData(
        trajectory_id_, carto::common::ToUniversal(time), topic,
        carto::common::make_unique<::cartographer::sensor::Data>(
            CheckNoLeadingSlash(msg->header.frame_id),
            carto::sensor::TransformLaserFan3D(
                carto::sensor::FromProto(ToCartographer(pcl_point_cloud)),
                sensor_to_tracking->cast<float>())));
  }
}

void SensorBridge::HandleLaserScanProto(
    const string& topic, const ::cartographer::common::Time time,
    const string& frame_id,
    const ::cartographer::sensor::proto::LaserScan& laser_scan) {
  const auto laser_fan_2d = carto::sensor::ToLaserFan(
      laser_scan, options_.horizontal_laser_min_range,
      options_.horizontal_laser_max_range,
      options_.horizontal_laser_missing_echo_ray_length);
  const auto sensor_to_tracking = tf_bridge_->LookupToTracking(time, frame_id);
  if (sensor_to_tracking != nullptr) {
    sensor_collator_->AddSensorData(
        trajectory_id_, carto::common::ToUniversal(time), topic,
        carto::common::make_unique<::cartographer::sensor::Data>(
            CheckNoLeadingSlash(frame_id),
            carto::sensor::TransformLaserFan3D(
                carto::sensor::ToLaserFan3D(laser_fan_2d),
                sensor_to_tracking->cast<float>())));
  }
}

}  // namespace cartographer_ros
