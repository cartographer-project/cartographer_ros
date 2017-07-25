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

#include "cartographer_rviz/per_trajectory_submap_display.h"

#include "cartographer/common/make_unique.h"

namespace cartographer_rviz {

namespace {
constexpr int kMaxOnGoingRequestsPerTrajectory = 6;

} // namespace

PerTrajectorySubmapDisplay::PerTrajectorySubmapDisplay(int trajectory_id,
                                       ::rviz::Property* submaps_category,
                                       ::rviz::DisplayContext* display_context,
                                       bool visible)
    : id_(trajectory_id),
      display_context_(display_context)
{
  visible_ = ::cartographer::common::make_unique<::rviz::BoolProperty>(
      QString("Trajectory %1").arg(id_), visible,
      QString("List of all submaps in Trajectory %1.").arg(id_),
      submaps_category, SLOT(AllEnabledToggled()), this);
}

PerTrajectorySubmapDisplay::~PerTrajectorySubmapDisplay() {
  submaps_.clear();
}

void PerTrajectorySubmapDisplay::AllEnabledToggled() {
  const bool visibility = visible_->getBool();
  for (auto& submap : submaps_) {
    submap.second->set_visibility(visibility);
  }
}

bool PerTrajectorySubmapDisplay::IsTrajectoryInvaild(
    const ::cartographer_ros_msgs::SubmapEntry& submap_entry)
{
  const auto it = submaps_.find(submap_entry.submap_index);
  return (it != submaps_.end() &&
        it->second->version() > submap_entry.submap_version);
}

void PerTrajectorySubmapDisplay::ProcessMessage(
    const ::std_msgs::Header& header,
    const ::cartographer_ros_msgs::SubmapEntry& submap_entry)
{
  const SubmapId id{submap_entry.trajectory_id, submap_entry.submap_index};

  if(submaps_.count(id.submap_index) == 0) {
    constexpr float kSubmapPoseAxesLength = 0.3f;
    constexpr float kSubmapPoseAxesRadius = 0.06f;
    submaps_.emplace(
        id.submap_index,
        ::cartographer::common::make_unique<DrawableSubmap>(
            id, display_context_, visible_.get(), visible_->getBool(),
            kSubmapPoseAxesLength, kSubmapPoseAxesRadius));
  }

  submaps_.at(id.submap_index)->Update(header, submap_entry,
                                       display_context_->getFrameManager());
}

void PerTrajectorySubmapDisplay::RemoveUnlistedSubmaps(
    const std::set<SubmapId> listed_submaps) {
  for (auto it = submaps_.begin(); it != submaps_.end();) {
    if (listed_submaps.count(
            SubmapId{static_cast<int>(id_), it->first}) == 0) {
      LOG(INFO) << "Removed";
      it = submaps_.erase(it);
    } else {
      ++it;
    }
  }
}

void PerTrajectorySubmapDisplay::SetAlpha(float current_tracking_z) {
  for (auto& submap : submaps_) {
    submap.second->SetAlpha(current_tracking_z);
  }
}

void PerTrajectorySubmapDisplay::FetchTexture(
    ros::ServiceClient* const client) {
  int num_ongoing_requests = 0;
  for (const auto& submap_entry : submaps_) {
    if (submap_entry.second->QueryInProgress()) {
      ++num_ongoing_requests;
    }
  }
  for (auto it = submaps_.rbegin();
       it != submaps_.rend() &&
       num_ongoing_requests < kMaxOnGoingRequestsPerTrajectory;
       ++it) {
    if (it->second->MaybeFetchTexture(client)) {
      ++num_ongoing_requests;
    }
  }
}

}  // namespace cartographer_rviz
