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
  tracking_frame_property_ = new ::rviz::StringProperty(
      "Tracking frame", kDefaultTrackingFrame,
      "Tracking frame, used for fading out submaps.", this);
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>("");
  trajectories_category_ = new ::rviz::Property(
      "Submaps", QVariant(), "List of all submaps, organized by trajectories.",
      this);
  visibility_all_enabled_ = new ::rviz::BoolProperty(
      "All Enabled", true,
      "Whether submaps from all trajectories should be displayed or not.",
      trajectories_category_, SLOT(AllEnabledToggled()), this);
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

SubmapsDisplay::~SubmapsDisplay() {
  client_.shutdown();
  trajectories_.clear();
  scene_manager_->destroySceneNode(map_node_);
}

void SubmapsDisplay::Reset() { reset(); }

void SubmapsDisplay::CreateClient() {
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
      submap_query_service_property_->getStdString());
}

void SubmapsDisplay::onInitialize() {
  MFDClass::onInitialize();
  map_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
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
  map_frame_ =
      ::cartographer::common::make_unique<std::string>(msg->header.frame_id);
  // In case Cartographer node is relaunched, destroy trajectories from the
  // previous instance.
  for (const ::cartographer_ros_msgs::SubmapEntry& submap_entry : msg->submap) {
    const size_t trajectory_id = submap_entry.trajectory_id;
    if (trajectory_id >= trajectories_.size()) {
      continue;
    }
    const auto& trajectory_submaps = trajectories_[trajectory_id]->submaps;
    const auto it = trajectory_submaps.find(submap_entry.submap_index);
    if (it != trajectory_submaps.end() &&
        it->second->version() > submap_entry.submap_version) {
      // Versions should only increase unless Cartographer restarted.
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
      trajectories_.push_back(::cartographer::common::make_unique<Trajectory>(
          ::cartographer::common::make_unique<::rviz::BoolProperty>(
              QString("Trajectory %1").arg(id.trajectory_id),
              visibility_all_enabled_->getBool(),
              QString("List of all submaps in Trajectory %1. The checkbox "
                      "controls whether all submaps in this trajectory should "
                      "be displayed or not.")
                  .arg(id.trajectory_id),
              trajectories_category_)));
    }
    auto& trajectory_visibility = trajectories_[id.trajectory_id]->visibility;
    auto& trajectory_submaps = trajectories_[id.trajectory_id]->submaps;
    if (trajectory_submaps.count(id.submap_index) == 0) {
      // TODO(ojura): Add RViz properties for adjusting submap pose axes
      constexpr float kSubmapPoseAxesLength = 0.3f;
      constexpr float kSubmapPoseAxesRadius = 0.06f;
      trajectory_submaps.emplace(
          id.submap_index,
          ::cartographer::common::make_unique<DrawableSubmap>(
              id, context_, map_node_, trajectory_visibility.get(),
              trajectory_visibility->getBool(), kSubmapPoseAxesLength,
              kSubmapPoseAxesRadius));
    }
    trajectory_submaps.at(id.submap_index)->Update(msg->header, submap_entry);
  }
  // Remove all submaps not mentioned in the SubmapList.
  for (size_t trajectory_id = 0; trajectory_id < trajectories_.size();
       ++trajectory_id) {
    auto& trajectory_submaps = trajectories_[trajectory_id]->submaps;
    for (auto it = trajectory_submaps.begin();
         it != trajectory_submaps.end();) {
      if (listed_submaps.count(
              SubmapId{static_cast<int>(trajectory_id), it->first}) == 0) {
        it = trajectory_submaps.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void SubmapsDisplay::update(const float wall_dt, const float ros_dt) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  // Schedule fetching of new submap textures.
  for (const auto& trajectory : trajectories_) {
    int num_ongoing_requests = 0;
    for (const auto& submap_entry : trajectory->submaps) {
      if (submap_entry.second->QueryInProgress()) {
        ++num_ongoing_requests;
      }
    }
    for (auto it = trajectory->submaps.rbegin();
         it != trajectory->submaps.rend() &&
         num_ongoing_requests < kMaxOnGoingRequestsPerTrajectory;
         ++it) {
      if (it->second->MaybeFetchTexture(&client_)) {
        ++num_ongoing_requests;
      }
    }
  }
  if (map_frame_ == nullptr) {
    return;
  }
  // Update the fading by z distance.
  const ros::Time kLatest(0);
  try {
    const ::geometry_msgs::TransformStamped transform_stamped =
        tf_buffer_.lookupTransform(
            *map_frame_, tracking_frame_property_->getStdString(), kLatest);
    for (auto& trajectory : trajectories_) {
      for (auto& submap_entry : trajectory->submaps) {
        submap_entry.second->SetAlpha(
            transform_stamped.transform.translation.z);
      }
    }
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1., "Could not compute submap fading: %s", ex.what());
  }
  // Update the map frame to fixed frame transform.
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (context_->getFrameManager()->getTransform(*map_frame_, kLatest, position,
                                                orientation)) {
    map_node_->setPosition(position);
    map_node_->setOrientation(orientation);
    context_->queueRender();
  }
}

void SubmapsDisplay::AllEnabledToggled() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  const bool visible = visibility_all_enabled_->getBool();
  for (auto& trajectory : trajectories_) {
    trajectory->visibility->setBool(visible);
  }
}

void Trajectory::AllEnabledToggled() {
  const bool visible = visibility->getBool();
  for (auto& submap_entry : submaps) {
    submap_entry.second->set_visibility(visible);
  }
}

Trajectory::Trajectory(std::unique_ptr<::rviz::BoolProperty> property)
    : visibility(std::move(property)) {
  ::QObject::connect(visibility.get(), SIGNAL(changed()), this,
                     SLOT(AllEnabledToggled()));
}

}  // namespace cartographer_rviz

PLUGINLIB_EXPORT_CLASS(cartographer_rviz::SubmapsDisplay, ::rviz::Display)
