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
#include "tf2_msgs/TFMessage.h"

namespace cartographer_ros {
namespace {

std::vector<::cartographer::transform::Rigid3d>
WaitForNTransformsWithChildFrameId(std::size_t num_transforms,
                                   const std::string& child_frame_id) {
  ::ros::NodeHandle node_handle;
  std::vector<::cartographer::transform::Rigid3d> result;
  auto callback = boost::function<void(const tf2_msgs::TFMessage&)>(
      [&result, child_frame_id](const tf2_msgs::TFMessage& transform) {
        for (const auto& t : transform.transforms) {
          LOG(INFO) << "Received tf message with child_frame_id: "
                    << t.child_frame_id;
          LOG(INFO) << "Received stamp:" << t.header.stamp;
          LOG(INFO) << "Received translation:" << t.transform.translation;
          if (t.child_frame_id == child_frame_id) {
            result.push_back(ToRigid3d(t));
          }
        }
      });
  ::ros::Subscriber subscriber =
      node_handle.subscribe<tf2_msgs::TFMessage>("tf", 1000, callback);
  ::ros::Rate rate(1000);
  while (result.size() < num_transforms) {
    ::ros::spinOnce();
    rate.sleep();
  }
  subscriber.shutdown();
  return result;
}

TEST(ResultSubscriberTest, ReceiveMovingBaseLink) {
  ::ros::init(std::map<std::string, std::string>{}, "result_subscriber_test");
  ScopedRosLogSink ros_log_sink;
  auto base_link_poses = WaitForNTransformsWithChildFrameId(1000, "base_link");
  const double kMinimumTravelDistance = 0.2;
  const double kMaximumTravelDistance = 1.0;
  EXPECT_GE((base_link_poses.back().translation() -
             base_link_poses.front().translation())
                .norm(),
            kMinimumTravelDistance);
  EXPECT_LE((base_link_poses.back().translation() -
             base_link_poses.front().translation())
                .norm(),
            kMaximumTravelDistance);
}

}  // namespace
}  // namespace cartographer_ros
