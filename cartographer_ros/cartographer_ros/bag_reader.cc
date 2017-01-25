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

#include "cartographer_ros/bag_reader.h"

#include <string>
#include <vector>

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_msgs/TFMessage.h"

namespace cartographer_ros {

constexpr char kTfStaticTopic[] = "/tf_static";

std::unique_ptr<tf2_ros::Buffer> ReadTransformsFromBag(
    const string& bag_filename) {
  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag);

  auto tf_buffer =
      ::cartographer::common::make_unique<tf2_ros::Buffer>(::ros::DURATION_MAX);
  const ::ros::Time begin_time = view.getBeginTime();
  const double duration_in_seconds = (view.getEndTime() - begin_time).toSec();
  for (const rosbag::MessageInstance& msg : view) {
    if (msg.isType<tf2_msgs::TFMessage>()) {
      const auto tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
      for (const auto& transform : tf_msg->transforms) {
        try {
          // TODO(damonkohler): Handle topic remapping.
          tf_buffer->setTransform(transform, "unused_authority",
                                  msg.getTopic() == kTfStaticTopic);
        } catch (const tf2::TransformException& ex) {
          LOG(WARNING) << ex.what();
        }
      }
    }
    LOG_EVERY_N(INFO, 100000)
        << "Processed " << (msg.getTime() - begin_time).toSec() << " of "
        << duration_in_seconds << " bag time seconds...";
  }

  bag.close();
  return tf_buffer;
}

}  // namespace cartographer_ros
