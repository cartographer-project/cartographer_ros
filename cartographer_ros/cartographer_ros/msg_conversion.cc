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

#include "cartographer_ros/msg_conversion.h"

#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

namespace {

// The ros::sensor_msgs::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::cartographer::transform::Rigid3d;
using ::cartographer::kalman_filter::PoseCovariance;

sensor_msgs::PointCloud2 PreparePointCloud2Message(const int64 timestamp,
                                                   const string& frame_id,
                                                   const int num_points) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = 7;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = 7;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = 7;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}

}  // namespace

sensor_msgs::MultiEchoLaserScan ToMultiEchoLaserScanMessage(
    const int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::LaserScan& laser_scan) {
  sensor_msgs::MultiEchoLaserScan msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;

  msg.angle_min = laser_scan.angle_min();
  msg.angle_max = laser_scan.angle_max();
  msg.angle_increment = laser_scan.angle_increment();
  msg.time_increment = laser_scan.time_increment();
  msg.scan_time = laser_scan.scan_time();
  msg.range_min = laser_scan.range_min();
  msg.range_max = laser_scan.range_max();

  for (const auto& echoes : laser_scan.range()) {
    msg.ranges.emplace_back();
    std::copy(echoes.value().begin(), echoes.value().end(),
              std::back_inserter(msg.ranges.back().echoes));
  }

  for (const auto& echoes : laser_scan.intensity()) {
    msg.intensities.emplace_back();
    std::copy(echoes.value().begin(), echoes.value().end(),
              std::back_inserter(msg.intensities.back().echoes));
  }
  return msg;
}

sensor_msgs::LaserScan ToLaserScan(
    const int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::LaserScan& laser_scan) {
  sensor_msgs::LaserScan msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;

  msg.angle_min = laser_scan.angle_min();
  msg.angle_max = laser_scan.angle_max();
  msg.angle_increment = laser_scan.angle_increment();
  msg.time_increment = laser_scan.time_increment();
  msg.scan_time = laser_scan.scan_time();
  msg.range_min = laser_scan.range_min();
  msg.range_max = laser_scan.range_max();

  for (const auto& echoes : laser_scan.range()) {
    if (echoes.value_size() > 0) {
      msg.ranges.push_back(echoes.value(0));
    } else {
      msg.ranges.push_back(0.);
    }
  }

  bool has_intensities = false;
  for (const auto& echoes : laser_scan.intensity()) {
    if (echoes.value_size() > 0) {
      msg.intensities.push_back(echoes.value(0));
      has_intensities = true;
    } else {
      msg.intensities.push_back(0);
    }
  }
  if (!has_intensities) {
    msg.intensities.clear();
  }

  return msg;
}

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::PointCloud& point_cloud) {
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (const auto& point : point_cloud) {
    stream.next(point.x());
    stream.next(point.y());
    stream.next(point.z());
    stream.next(kPointCloudComponentFourMagic);
  }
  return msg;
}

sensor_msgs::PointCloud2 ToPointCloud2Message(
    const int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::proto::RangeData& range_data) {
  CHECK(::cartographer::transform::ToEigen(range_data.origin()).norm() == 0)
      << "Trying to convert a range_data that is not at the origin.";

  const auto& point_cloud = range_data.point_cloud();
  CHECK_EQ(point_cloud.x_size(), point_cloud.y_size());
  CHECK_EQ(point_cloud.x_size(), point_cloud.z_size());
  const auto num_points = point_cloud.x_size();

  auto msg = PreparePointCloud2Message(timestamp, frame_id, num_points);
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (int i = 0; i < num_points; ++i) {
    stream.next(point_cloud.x(i));
    stream.next(point_cloud.y(i));
    stream.next(point_cloud.z(i));
    stream.next(kPointCloudComponentFourMagic);
  }
  return msg;
}

::cartographer::sensor::proto::LaserScan ToCartographer(
    const sensor_msgs::LaserScan& msg) {
  ::cartographer::sensor::proto::LaserScan proto;
  proto.set_angle_min(msg.angle_min);
  proto.set_angle_max(msg.angle_max);
  proto.set_angle_increment(msg.angle_increment);
  proto.set_time_increment(msg.time_increment);
  proto.set_scan_time(msg.scan_time);
  proto.set_range_min(msg.range_min);
  proto.set_range_max(msg.range_max);

  for (const auto& range : msg.ranges) {
    proto.add_range()->mutable_value()->Add(range);
  }

  for (const auto& intensity : msg.intensities) {
    proto.add_intensity()->mutable_value()->Add(intensity);
  }
  return proto;
}

::cartographer::sensor::proto::LaserScan ToCartographer(
    const sensor_msgs::MultiEchoLaserScan& msg) {
  ::cartographer::sensor::proto::LaserScan proto;
  proto.set_angle_min(msg.angle_min);
  proto.set_angle_max(msg.angle_max);
  proto.set_angle_increment(msg.angle_increment);
  proto.set_time_increment(msg.time_increment);
  proto.set_scan_time(msg.scan_time);
  proto.set_range_min(msg.range_min);
  proto.set_range_max(msg.range_max);

  for (const auto& range : msg.ranges) {
    auto* proto_echoes = proto.add_range()->mutable_value();
    for (const auto& echo : range.echoes) {
      proto_echoes->Add(echo);
    }
  }
  for (const auto& intensity : msg.intensities) {
    auto* proto_echoes = proto.add_intensity()->mutable_value();
    for (const auto& echo : intensity.echoes) {
      proto_echoes->Add(echo);
    }
  }
  return proto;
}

Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

PoseCovariance ToPoseCovariance(const boost::array<double, 36>& covariance) {
  return Eigen::Map<const Eigen::Matrix<double, 6, 6>>(covariance.data());
}

geometry_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
  geometry_msgs::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = rigid3d.translation().z();
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}

geometry_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
  geometry_msgs::Pose pose;
  pose.position.x = rigid3d.translation().x();
  pose.position.y = rigid3d.translation().y();
  pose.position.z = rigid3d.translation().z();
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

}  // namespace cartographer_ros
