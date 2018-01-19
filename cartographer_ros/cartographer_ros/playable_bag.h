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
#include <functional>
#include <queue>

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

class PlayableBag {
 public:
  using BufferCallback = std::function<bool(const rosbag::MessageInstance&)>;

  PlayableBag(const std::string& bag_filename, int bag_id, ros::Time start_time,
              ros::Time end_time, ros::Duration buffer_delay,
              BufferCallback buffer_callback);

  ros::Time PeekMessageTime();

  rosbag::MessageInstance GetNextMessage();

  bool IsMessageAvailable();
  int bag_id();

 private:
  void AdvanceOneMessage();

  void AdvanceUntilMessageAvailable();

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> view_;

  rosbag::View::const_iterator view_iterator_;

  bool finished_;
  const int bag_id_;

  const std::string bag_filename_;

  const double duration_in_seconds_;
  int log_counter_;

  std::deque<rosbag::MessageInstance> buffered_messages_;

  const ::ros::Duration buffer_delay_;
  BufferCallback buffer_callback_;
};

class PlayableBagMultiplexer {
 public:
  void AddPlayableBag(PlayableBag playable_bag);
  bool IsMessageAvailable();

  // Returns the next message from the multiplexed (merge-sorted) message
  // stream, along with the bag id corresponding to the message, and whether
  // this was the last message in that bag.
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
