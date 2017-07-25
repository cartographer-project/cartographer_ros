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

PerTrajectorySubmapDisplay::PerTrajectorySubmapDisplay(const int trajectory_id,
                                       ::rviz::Property* submap_category,
                                       ::rviz::DisplayContext* display_context,
                                       const bool visible)
    : id_(trajectory_id),
      submaps_category_(submap_category),
      display_context_(display_context)
{
  property_ = new ::rviz::BoolProperty(
      QString("Trajectory %1").arg(id_), visible,
      QString("List of all submaps in Trajectory %1.").arg(id_),
      submaps_category_, SLOT(AllEnabledToggled()), this);
}

PerTrajectorySubmapDisplay::~PerTrajectorySubmapDisplay() {
  submaps_.clear();
  delete property_;
}

void PerTrajectorySubmapDisplay::AllEnabledToggled() {
  const bool visibility = property_->getBool();
  for (auto& submap : submaps_) {
    submap.second->set_visibility(visibility);
  }
}

void PerTrajectorySubmapDisplay::AddSubmap(
    const ::cartographer::mapping::SubmapId& submap_id) {
  constexpr float kSubmapPoseAxesLength = 0.3f;
  constexpr float kSubmapPoseAxesRadius = 0.06f;
  submaps_.emplace(
      submap_id.submap_index,
      ::cartographer::common::make_unique<DrawableSubmap>(
          submap_id, display_context_, property_, property_->getBool(),
          kSubmapPoseAxesLength, kSubmapPoseAxesRadius));
}

void PerTrajectorySubmapDisplay::UpdateSubmap(
    const int submap_index, const ::std_msgs::Header& header,
    const ::cartographer_ros_msgs::SubmapEntry& metadata,
    ::rviz::FrameManager* frame_manager) {
  submaps_.at(submap_index)->Update(header, metadata, frame_manager);
}
}
