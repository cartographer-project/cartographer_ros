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

#include "absl/memory/memory.h"
#include "cartographer_ros/node_constants.h"
#include "glog/logging.h"
#include "tf2_msgs/TFMessage.h"

namespace cartographer_ros {

PlayableBag::PlayableBag(
    const std::string& bag_filename, const int bag_id,
    const ros::Time start_time, const ros::Time end_time,
    const ros::Duration buffer_delay,
    FilteringEarlyMessageHandler filtering_early_message_handler)
    : bag_(absl::make_unique<rosbag::Bag>(bag_filename, rosbag::bagmode::Read)),
      view_(absl::make_unique<rosbag::View>(*bag_, start_time, end_time)),
      view_iterator_(view_->begin()),
      finished_(false),
      bag_id_(bag_id),
      bag_filename_(bag_filename),
      duration_in_seconds_(
          (view_->getEndTime() - view_->getBeginTime()).toSec()),
      message_counter_(0),
      buffer_delay_(buffer_delay),
      filtering_early_message_handler_(
          std::move(filtering_early_message_handler)) {
  AdvanceUntilMessageAvailable();
  for (const auto* connection_info : view_->getConnections()) {
    topics_.insert(connection_info->topic);
  }
}

ros::Time PlayableBag::PeekMessageTime() const {
  CHECK(IsMessageAvailable());
  return buffered_messages_.front().getTime();
}

std::tuple<ros::Time, ros::Time> PlayableBag::GetBeginEndTime() const {
  return std::make_tuple(view_->getBeginTime(), view_->getEndTime());
}

rosbag::MessageInstance PlayableBag::GetNextMessage(
    cartographer_ros_msgs::BagfileProgress* progress) {
  CHECK(IsMessageAvailable());
  const rosbag::MessageInstance msg = buffered_messages_.front();
  buffered_messages_.pop_front();
  AdvanceUntilMessageAvailable();
  double processed_seconds = (msg.getTime() - view_->getBeginTime()).toSec();
  if ((message_counter_ % 10000) == 0) {
    LOG(INFO) << "Processed " << processed_seconds << " of "
              << duration_in_seconds_ << " seconds of bag " << bag_filename_;
  }

  if (progress) {
    progress->current_bagfile_name = bag_filename_;
    progress->current_bagfile_id = bag_id_;
    progress->total_messages = view_->size();
    progress->processed_messages = message_counter_;
    progress->total_seconds = duration_in_seconds_;
    progress->processed_seconds = processed_seconds;
  }

  return msg;
}

bool PlayableBag::IsMessageAvailable() const {
  return !buffered_messages_.empty() &&
         (buffered_messages_.front().getTime() <
          buffered_messages_.back().getTime() - buffer_delay_);
}

int PlayableBag::bag_id() const { return bag_id_; }

void PlayableBag::AdvanceOneMessage() {
  CHECK(!finished_);
  if (view_iterator_ == view_->end()) {
    finished_ = true;
    return;
  }
  rosbag::MessageInstance& msg = *view_iterator_;
  if (!filtering_early_message_handler_ ||
      filtering_early_message_handler_(msg)) {
    buffered_messages_.push_back(msg);
  }
  ++view_iterator_;
  ++message_counter_;
}

void PlayableBag::AdvanceUntilMessageAvailable() {
  if (finished_) {
    return;
  }
  do {
    AdvanceOneMessage();
  } while (!finished_ && !IsMessageAvailable());
}

PlayableBagMultiplexer::PlayableBagMultiplexer() : pnh_("~") {
  bag_progress_pub_ = pnh_.advertise<cartographer_ros_msgs::BagfileProgress>(
      "bagfile_progress", 10);
  progress_pub_interval_ = pnh_.param("bagfile_progress_pub_interval", 10.0);
}

void PlayableBagMultiplexer::AddPlayableBag(PlayableBag playable_bag) {
  for (const auto& topic : playable_bag.topics()) {
    topics_.insert(topic);
  }
  playable_bags_.push_back(std::move(playable_bag));
  CHECK(playable_bags_.back().IsMessageAvailable());
  next_message_queue_.emplace(
      BagMessageItem{playable_bags_.back().PeekMessageTime(),
                     static_cast<int>(playable_bags_.size() - 1)});
  bag_progress_time_map_[playable_bag.bag_id()] = ros::Time::now();
}

bool PlayableBagMultiplexer::IsMessageAvailable() const {
  return !next_message_queue_.empty();
}

std::tuple<rosbag::MessageInstance, int, bool>
PlayableBagMultiplexer::GetNextMessage() {
  CHECK(IsMessageAvailable());
  const int current_bag_index = next_message_queue_.top().bag_index;
  PlayableBag& current_bag = playable_bags_.at(current_bag_index);
  cartographer_ros_msgs::BagfileProgress progress;
  rosbag::MessageInstance msg = current_bag.GetNextMessage(&progress);
  const bool publish_progress =
      current_bag.finished() ||
      ros::Time::now() - bag_progress_time_map_[current_bag.bag_id()] >=
          ros::Duration(progress_pub_interval_);
  if (bag_progress_pub_.getNumSubscribers() > 0 && publish_progress) {
    progress.total_bagfiles = playable_bags_.size();
    if (current_bag.finished()) {
      progress.processed_seconds = current_bag.duration_in_seconds();
    }
    bag_progress_pub_.publish(progress);
    bag_progress_time_map_[current_bag.bag_id()] = ros::Time::now();
  }
  CHECK_EQ(msg.getTime(), next_message_queue_.top().message_timestamp);
  next_message_queue_.pop();
  if (current_bag.IsMessageAvailable()) {
    next_message_queue_.emplace(
        BagMessageItem{current_bag.PeekMessageTime(), current_bag_index});
  }
  return std::make_tuple(std::move(msg), current_bag.bag_id(),
                         !current_bag.IsMessageAvailable());
}

ros::Time PlayableBagMultiplexer::PeekMessageTime() const {
  CHECK(IsMessageAvailable());
  return next_message_queue_.top().message_timestamp;
}

}  // namespace cartographer_ros
