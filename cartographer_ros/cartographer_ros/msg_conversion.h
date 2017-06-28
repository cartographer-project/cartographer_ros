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

#ifndef CARTOGRAPHER_ROS_MSG_CONVERSION_H_
#define CARTOGRAPHER_ROS_MSG_CONVERSION_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

sensor_msgs::PointCloud2 ToPointCloud2Message(
    int64 timestamp, const string& frame_id,
    const ::cartographer::sensor::PointCloud& point_cloud);

geometry_msgs::Transform ToGeometryMsgTransform(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::Pose ToGeometryMsgPose(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::LaserScan& msg);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::MultiEchoLaserScan& msg);

::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(
    const sensor_msgs::PointCloud2& message);

::cartographer::transform::Rigid3d ToRigid3d(
    const geometry_msgs::TransformStamped& transform);

::cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_MSG_CONVERSION_H_
