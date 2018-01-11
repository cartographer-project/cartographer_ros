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

#include <cmath>

#include "gtest/gtest.h"
#include "sensor_msgs/LaserScan.h"

namespace cartographer_ros {
namespace {

TEST(MsgConversion, LaserScanToPointCloud) {
  sensor_msgs::LaserScan laser_scan;
  for (int i = 0; i < 8; ++i) {
    laser_scan.ranges.push_back(1.f);
  }
  laser_scan.angle_min = 0.f;
  laser_scan.angle_max = 8.f * static_cast<float>(M_PI_4);
  laser_scan.angle_increment = static_cast<float>(M_PI_4);
  laser_scan.range_min = 0.f;
  laser_scan.range_max = 10.f;

  const auto point_cloud =
      std::get<0>(ToPointCloudWithIntensities(laser_scan)).points;
  EXPECT_TRUE(
      point_cloud[0].isApprox(Eigen::Vector4f(1.f, 0.f, 0.f, 0.f), 1e-6));
  EXPECT_TRUE(point_cloud[1].isApprox(
      Eigen::Vector4f(1.f / std::sqrt(2.f), 1.f / std::sqrt(2.f), 0.f, 0.f),
      1e-6));
  EXPECT_TRUE(
      point_cloud[2].isApprox(Eigen::Vector4f(0.f, 1.f, 0.f, 0.f), 1e-6));
  EXPECT_TRUE(point_cloud[3].isApprox(
      Eigen::Vector4f(-1.f / std::sqrt(2.f), 1.f / std::sqrt(2.f), 0.f, 0.f),
      1e-6));
  EXPECT_TRUE(
      point_cloud[4].isApprox(Eigen::Vector4f(-1.f, 0.f, 0.f, 0.f), 1e-6));
  EXPECT_TRUE(point_cloud[5].isApprox(
      Eigen::Vector4f(-1.f / std::sqrt(2.f), -1.f / std::sqrt(2.f), 0.f, 0.f),
      1e-6));
  EXPECT_TRUE(
      point_cloud[6].isApprox(Eigen::Vector4f(0.f, -1.f, 0.f, 0.f), 1e-6));
  EXPECT_TRUE(point_cloud[7].isApprox(
      Eigen::Vector4f(1.f / std::sqrt(2.f), -1.f / std::sqrt(2.f), 0.f, 0.f),
      1e-6));
}

TEST(MsgConversion, LaserScanToPointCloudWithInfinityAndNaN) {
  sensor_msgs::LaserScan laser_scan;
  laser_scan.ranges.push_back(1.f);
  laser_scan.ranges.push_back(std::numeric_limits<float>::infinity());
  laser_scan.ranges.push_back(2.f);
  laser_scan.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
  laser_scan.ranges.push_back(3.f);
  laser_scan.angle_min = 0.f;
  laser_scan.angle_max = 3.f * static_cast<float>(M_PI_4);
  laser_scan.angle_increment = static_cast<float>(M_PI_4);
  laser_scan.range_min = 2.f;
  laser_scan.range_max = 10.f;

  const auto point_cloud =
      std::get<0>(ToPointCloudWithIntensities(laser_scan)).points;
  ASSERT_EQ(2, point_cloud.size());
  EXPECT_TRUE(
      point_cloud[0].isApprox(Eigen::Vector4f(0.f, 2.f, 0.f, 0.f), 1e-6));
  EXPECT_TRUE(
      point_cloud[1].isApprox(Eigen::Vector4f(-3.f, 0.f, 0.f, 0.f), 1e-6));
}

TEST(MsgConversion, LatLongAltToEcef) {
  Eigen::Vector3d equator_prime_meridian = LatLongAltToEcef(0, 0, 0);
  EXPECT_TRUE(equator_prime_meridian.isApprox(Eigen::Vector3d(6378137, 0, 0)))
      << equator_prime_meridian;
  Eigen::Vector3d plus_ten_meters = LatLongAltToEcef(0, 0, 10);
  EXPECT_TRUE(plus_ten_meters.isApprox(Eigen::Vector3d(6378147, 0, 0)))
      << plus_ten_meters;
  Eigen::Vector3d north_pole = LatLongAltToEcef(90, 0, 0);
  EXPECT_TRUE(north_pole.isApprox(Eigen::Vector3d(0, 0, 6356752.3142), 1e-9))
      << north_pole;
  Eigen::Vector3d also_north_pole = LatLongAltToEcef(90, 90, 0);
  EXPECT_TRUE(
      also_north_pole.isApprox(Eigen::Vector3d(0, 0, 6356752.3142), 1e-9))
      << also_north_pole;
  Eigen::Vector3d south_pole = LatLongAltToEcef(-90, 0, 0);
  EXPECT_TRUE(south_pole.isApprox(Eigen::Vector3d(0, 0, -6356752.3142), 1e-9))
      << south_pole;
  Eigen::Vector3d above_south_pole = LatLongAltToEcef(-90, 60, 20);
  EXPECT_TRUE(
      above_south_pole.isApprox(Eigen::Vector3d(0, 0, -6356772.3142), 1e-9))
      << above_south_pole;
  Eigen::Vector3d somewhere_on_earth =
      LatLongAltToEcef(48.1372149, 11.5748024, 517.1);
  EXPECT_TRUE(somewhere_on_earth.isApprox(
      Eigen::Vector3d(4177983, 855702, 4727457), 1e-6))
      << somewhere_on_earth;
}

}  // namespace
}  // namespace cartographer_ros
