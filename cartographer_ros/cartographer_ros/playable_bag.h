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

#ifndef CARTOGRAPHER_ROS_PLAYABLE_BAG_H_
#define CARTOGRAPHER_ROS_PLAYABLE_BAG_H_
#include <queue>

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

class PlayableBag {
 public:
  PlayableBag(const std::string bag_filename, tf2_ros::Buffer* const tf_buffer,
              ros::Publisher* const tf_publisher, const int trajectory_id,
              const bool use_bag_transforms);

  ros::Time PeekMessageTime();

  rosbag::MessageInstance GetNextMessage();

  bool IsMessageAvailable();
  int trajectory_id();

 private:
  void AdvanceOneMessage();

  void AdvanceUntilMessageAvailable();

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> view_;

  rosbag::View::const_iterator view_iterator_;
  tf2_ros::Buffer* const tf_buffer_;
  ::ros::Publisher* const tf_publisher_;

  bool finished_;
  const int trajectory_id_;

  const std::string bag_filename_;

  const double duration_in_seconds_;
  int log_counter_;

  // We need to keep 'tf_buffer' small because it becomes very inefficient
  // otherwise. We make sure that tf_messages are published before any data
  // messages, so that tf lookups always work.
  std::deque<rosbag::MessageInstance> delayed_messages_;
  const bool use_bag_transforms_;
  // We publish tf messages one second earlier than other messages. Under
  // the assumption of higher frequency tf this should ensure that tf can
  // always interpolate.
  const ::ros::Duration kDelay = ::ros::Duration(1.);
};

class PlayableBagMultiplexer {
 public:
  void AddPlayableBag(PlayableBag playable_bag);
  bool IsMessageAvailable();

  // Returns the next message from the multiplexed stream, along with the
  // id of the trajectory corresponding to the message.
  std::tuple<int, rosbag::MessageInstance, bool> GetNextMessage();

 private:
  std::vector<PlayableBag> playable_bags_;
  struct NextMessageTimestamp {
    ros::Time next_message_timestamp;
    int playable_bag_index;
  };
  struct cmp {
    bool operator()(const NextMessageTimestamp& l,
                    const NextMessageTimestamp& r) {
      return l.next_message_timestamp > r.next_message_timestamp;
    }
  };
  std::priority_queue<NextMessageTimestamp, std::vector<NextMessageTimestamp>,
                      cmp>
      next_message_queue_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_PLAYABLE_BAG_H_
