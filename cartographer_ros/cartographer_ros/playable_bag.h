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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H
#include <functional>
#include <queue>

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

class PlayableBag {
 public:
  // Handles messages early, i.e. when they are about to enter the buffer.
  // Returns a boolean indicating whether the message should enter the buffer.
  using FilteringEarlyMessageHandler =
      std::function<bool /* forward_message_to_buffer */ (
          const rosbag::MessageInstance&)>;

  PlayableBag(const std::string& bag_filename, int bag_id, ros::Time start_time,
              ros::Time end_time, ros::Duration buffer_delay,
              FilteringEarlyMessageHandler filtering_early_message_handler);

  ros::Time PeekMessageTime() const;
  rosbag::MessageInstance GetNextMessage();
  bool IsMessageAvailable() const;
  std::tuple<ros::Time, ros::Time> GetBeginEndTime() const;

  int bag_id() const;
  std::set<std::string> topics() const { return topics_; }

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
  FilteringEarlyMessageHandler filtering_early_message_handler_;
  std::set<std::string> topics_;
};

class PlayableBagMultiplexer {
 public:
  void AddPlayableBag(PlayableBag playable_bag);

  // Returns the next message from the multiplexed (merge-sorted) message
  // stream, along with the bag id corresponding to the message, and whether
  // this was the last message in that bag.
  std::tuple<rosbag::MessageInstance, int /* bag_id */,
             bool /* is_last_message_in_bag */>
  GetNextMessage();

  bool IsMessageAvailable() const;
  ros::Time PeekMessageTime() const;

  std::set<std::string> topics() const { return topics_; }

 private:
  struct BagMessageItem {
    ros::Time message_timestamp;
    int bag_index;
    struct TimestampIsGreater {
      bool operator()(const BagMessageItem& l, const BagMessageItem& r) {
        return l.message_timestamp > r.message_timestamp;
      }
    };
  };

  std::vector<PlayableBag> playable_bags_;
  std::priority_queue<BagMessageItem, std::vector<BagMessageItem>,
                      BagMessageItem::TimestampIsGreater>
      next_message_queue_;
  std::set<std::string> topics_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_PLAYABLE_BAG_H
