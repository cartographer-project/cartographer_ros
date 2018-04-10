/*
 * Copyright 2018 The Cartographer Authors
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

#include <string>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "geometry_msgs/TransformStamped.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros_log_sink.h"
#include "tf/transform_listener.h"

namespace cartographer_ros {
namespace {

TEST(ResultSubscriberTest, ReceiveMovingBaseLink) {
  ::ros::init(std::map<std::string, std::string>{}, "result_subscriber_test");
  ScopedRosLogSink ros_log_sink;
  ros::start();
  ::ros::NodeHandle node_handle;
  ::tf2_ros::Buffer buffer(ros::Duration(60));
  ::tf2_ros::TransformListener listener(buffer);
  std::vector<::cartographer::transform::Rigid3d> result_transforms;
  std::vector<geometry_msgs::TransformStamped> result_messages;
  const ros::Duration kSimulationRunTime(5);
  const ros::WallDuration kMaximumWaitTime(20);
  ::ros::Rate rate(100);
  ::ros::WallTime wall_time_start = ros::WallTime::now();
  ::ros::Duration observed_duration(0);
  while (observed_duration < kSimulationRunTime &&
         ros::WallTime::now() < wall_time_start + kMaximumWaitTime) {
    rate.sleep();
    try {
      auto transform = buffer.lookupTransform("map", "base_link", ros::Time(0));
      result_transforms.push_back(ToRigid3d(transform));
      result_messages.push_back(transform);
      observed_duration = result_messages.back().header.stamp -
                          result_messages.front().header.stamp;
    } catch (tf2::TransformException& e) {
      LOG(INFO) << e.what();
    }
  }
  EXPECT_GE(result_messages.size(), 10)
      << "Did not receive enough map to base_link transforms.";
  EXPECT_NEAR(observed_duration.toSec(), kSimulationRunTime.toSec(), 2.0);
  ASSERT_GT(result_transforms.size(), 0);
  double travel_distance = (result_transforms.back().translation() -
                            result_transforms.front().translation())
                               .norm();
  LOG(INFO) << "Observed simulation time:" << observed_duration;
  LOG(INFO) << "Observed travel distance:" << travel_distance;
  // TODO(gaschler): gtest cannot read arguments, so the expected range needs to
  // be large.
  const double kMinimumTravelDistance = 0.2;
  const double kMaximumTravelDistance = 5.0;
  EXPECT_GE(travel_distance, kMinimumTravelDistance);
  EXPECT_LE(travel_distance, kMaximumTravelDistance);
}

}  // namespace
}  // namespace cartographer_ros
