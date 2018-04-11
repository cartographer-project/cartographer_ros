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
  ::ros::Time::init();
  ::ros::start();
  ::ros::NodeHandle node_handle;
  EXPECT_TRUE(ros::service::waitForService("submap_query", ros::Duration(3)));
}

}  // namespace
}  // namespace cartographer_ros
