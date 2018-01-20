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

#include "cartographer_ros/playable_bag.h"

#include "cartographer/common/make_unique.h"
#include "cartographer_ros/node_constants.h"
#include "glog/logging.h"
#include "tf2_msgs/TFMessage.h"

namespace cartographer_ros {

PlayableBag::PlayableBag(const std::string& bag_filename, const int bag_id,
                         const ros::Time start_time, const ros::Time end_time,
                         const ros::Duration buffer_delay,
                         BufferCallback buffer_callback)
    : bag_(cartographer::common::make_unique<rosbag::Bag>(
          bag_filename, rosbag::bagmode::Read)),
      view_(cartographer::common::make_unique<rosbag::View>(*bag_, start_time,
                                                            end_time)),
      view_iterator_(view_->begin()),
      finished_(false),
      bag_id_(bag_id),
      bag_filename_(bag_filename),
      duration_in_seconds_(
          (view_->getEndTime() - view_->getBeginTime()).toSec()),
      log_counter_(0),
      buffer_delay_(buffer_delay),
      buffer_callback_(std::move(buffer_callback)) {
  AdvanceUntilMessageAvailable();
}

ros::Time PlayableBag::PeekMessageTime() {
  CHECK(IsMessageAvailable());
  return buffered_messages_.front().getTime();
}

std::pair<ros::Time, ros::Time> PlayableBag::GetBeginEndTime() {
  return std::make_pair(view_->getBeginTime(), view_->getEndTime());
}

rosbag::MessageInstance PlayableBag::GetNextMessage() {
  CHECK(IsMessageAvailable());
  const rosbag::MessageInstance msg = buffered_messages_.front();
  buffered_messages_.pop_front();
  AdvanceUntilMessageAvailable();
  if ((log_counter_++ % 10000) == 0) {
    LOG(INFO) << "Processed " << (msg.getTime() - view_->getBeginTime()).toSec()
              << " of " << duration_in_seconds_ << " seconds of bag "
              << bag_filename_;
  }
  return msg;
}

bool PlayableBag::IsMessageAvailable() {
  return !buffered_messages_.empty() &&
         (buffered_messages_.front().getTime() <
          buffered_messages_.back().getTime() - buffer_delay_);
}

int PlayableBag::bag_id() { return bag_id_; }

void PlayableBag::AdvanceOneMessage() {
  CHECK(!finished_);
  if (view_iterator_ == view_->end()) {
    finished_ = true;
    return;
  }
  rosbag::MessageInstance& msg = *view_iterator_;
  if (!buffer_callback_ || buffer_callback_(msg)) {
    buffered_messages_.push_back(msg);
  }
  ++view_iterator_;
}

void PlayableBag::AdvanceUntilMessageAvailable() {
  if (finished_) {
    return;
  }
  do {
    AdvanceOneMessage();
  } while (!finished_ && !IsMessageAvailable());
}

void PlayableBagMultiplexer::AddPlayableBag(PlayableBag playable_bag) {
  playable_bags_.push_back(std::move(playable_bag));
  CHECK(playable_bags_.back().IsMessageAvailable());
  next_message_queue_.emplace(
      NextMessageTimestamp{playable_bags_.back().PeekMessageTime(),
                           static_cast<int>(playable_bags_.size() - 1)});
}
bool PlayableBagMultiplexer::IsMessageAvailable() {
  return !next_message_queue_.empty();
}

std::tuple<rosbag::MessageInstance, int, bool>
PlayableBagMultiplexer::GetNextMessage() {
  CHECK(IsMessageAvailable());
  const int bag_index = next_message_queue_.top().playable_bag_index;
  rosbag::MessageInstance msg = playable_bags_.at(bag_index).GetNextMessage();
  CHECK_EQ(msg.getTime(), next_message_queue_.top().next_message_timestamp);
  next_message_queue_.pop();
  if (playable_bags_.at(bag_index).IsMessageAvailable()) {
    next_message_queue_.emplace(NextMessageTimestamp{
        playable_bags_.at(bag_index).PeekMessageTime(), bag_index});
  }
  return std::make_tuple(std::move(msg), playable_bags_.at(bag_index).bag_id(),
                         playable_bags_.at(bag_index).IsMessageAvailable());
}

}  // namespace cartographer_ros
