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

#include "cartographer_rviz/submaps_display.h"

#include "OgreResourceGroupManager.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/id.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "geometry_msgs/TransformStamped.h"
#include "pluginlib/class_list_macros.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/string_property.h"

namespace cartographer_rviz {

namespace {

constexpr int kMaxOnGoingRequestsPerTrajectory = 6;
constexpr char kMaterialsDirectory[] = "/ogre_media/materials";
constexpr char kGlsl120Directory[] = "/glsl120";
constexpr char kScriptsDirectory[] = "/scripts";
constexpr char kDefaultMapFrame[] = "map";
constexpr char kDefaultTrackingFrame[] = "base_link";
constexpr char kDefaultSubmapQueryServiceName[] = "/submap_query";

}  // namespace

SubmapsDisplay::SubmapsDisplay() : tf_listener_(tf_buffer_) {
  submap_query_service_property_ = new ::rviz::StringProperty(
      "Submap query service", kDefaultSubmapQueryServiceName,
      "Submap query service to connect to.", this, SLOT(Reset()));
  map_frame_property_ = new ::rviz::StringProperty(
      "Map frame", kDefaultMapFrame, "Map frame, used for fading out submaps.",
      this);
  tracking_frame_property_ = new ::rviz::StringProperty(
      "Tracking frame", kDefaultTrackingFrame,
      "Tracking frame, used for fading out submaps.", this);
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>("");
  submaps_category_ = new ::rviz::Property(
      "Submaps", QVariant(), "List of all submaps, organized by trajectories.",
      this);
  visibility_all_enabled_ = new ::rviz::BoolProperty(
      "All Enabled", true,
      "Whether all the submaps should be displayed or not.", submaps_category_,
      SLOT(AllEnabledToggled()), this);
  const std::string package_path = ::ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory, "FileSystem", ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kGlsl120Directory, "FileSystem",
      ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kScriptsDirectory, "FileSystem",
      ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

SubmapsDisplay::~SubmapsDisplay() { client_.shutdown(); }

void SubmapsDisplay::Reset() { reset(); }

void SubmapsDisplay::CreateClient() {
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
      submap_query_service_property_->getStdString());
}

void SubmapsDisplay::onInitialize() {
  MFDClass::onInitialize();
  CreateClient();
}

void SubmapsDisplay::reset() {
  MFDClass::reset();
  ::cartographer::common::MutexLocker locker(&mutex_);
  client_.shutdown();
  trajectories_.clear();
  CreateClient();
}

void SubmapsDisplay::processMessage(
    const ::cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  // In case Cartographer node is relaunched, destroy trajectories from the
  // previous instance.
  for (const ::cartographer_ros_msgs::SubmapEntry& submap_entry : msg->submap) {
    const size_t trajectory_id = submap_entry.trajectory_id;
    if (trajectory_id >= trajectories_.size()) {
      continue;
    }

    if (trajectories_[trajectory_id]->IsTrajectoryInvaild(submap_entry)) {
      trajectories_.clear();
      break;
    }
  }

  using ::cartographer::mapping::SubmapId;
  std::set<SubmapId> listed_submaps;
  for (const ::cartographer_ros_msgs::SubmapEntry& submap_entry : msg->submap) {
    const SubmapId id{submap_entry.trajectory_id, submap_entry.submap_index};
    listed_submaps.insert(id);
    while (id.trajectory_id >= static_cast<int>(trajectories_.size())) {
      trajectories_.push_back(
          ::cartographer::common::make_unique<PerTrajectorySubmapDisplay>(
              id.trajectory_id, submaps_category_, context_,
              visibility_all_enabled_->getBool()));
    }
    trajectories_[id.trajectory_id]->ProcessMessage(msg->header, submap_entry);
  }

  // Remove all submaps not mentioned in the SubmapList.
  for (size_t trajectory_id = 0; trajectory_id < trajectories_.size();
       ++trajectory_id) {
    trajectories_[trajectory_id]->RemoveUnlistedSubmaps(listed_submaps);
  }
}

void SubmapsDisplay::update(const float wall_dt, const float ros_dt) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  // Update the fading by z distance.
  try {
    const ::geometry_msgs::TransformStamped transform_stamped =
        tf_buffer_.lookupTransform(map_frame_property_->getStdString(),
                                   tracking_frame_property_->getStdString(),
                                   ros::Time(0) /* latest */);
    for (auto& trajectory : trajectories_) {
      trajectory->SetAlpha(transform_stamped.transform.translation.z);
    }
  } catch (const tf2::TransformException& ex) {
    ROS_WARN("Could not compute submap fading: %s", ex.what());
  }

  // Schedule fetching of new submap textures.
  for (const auto& trajectory : trajectories_) {
    trajectory->FetchTexture(&client_);
  }
}

void SubmapsDisplay::AllEnabledToggled() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  const bool visibility = visibility_all_enabled_->getBool();
  for (auto& trajectory : trajectories_) {
    trajectory->set_visibility(visibility);
  }
}

}  // namespace cartographer_rviz

PLUGINLIB_EXPORT_CLASS(cartographer_rviz::SubmapsDisplay, ::rviz::Display)
