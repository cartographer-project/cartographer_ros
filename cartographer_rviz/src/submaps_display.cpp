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
#include "absl/memory/memory.h"
#include "absl/synchronization/mutex.h"

#include "cartographer/mapping/id.h"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace cartographer_rviz {

namespace {

constexpr int kMaxOnGoingRequestsPerTrajectory = 6;
constexpr char kMaterialsDirectory[] = "/ogre_media/materials";
constexpr char kGlsl120Directory[] = "/glsl120";
constexpr char kScriptsDirectory[] = "/scripts";
constexpr char kDefaultTrackingFrame[] = "base_link";
constexpr char kDefaultSubmapQueryServiceName[] = "/submap_query";

}  // namespace

SubmapsDisplay::SubmapsDisplay() : rclcpp::Node("submaps_display") {
  submap_query_service_property_ = new ::rviz_common::properties::StringProperty(
      "Submap query service", kDefaultSubmapQueryServiceName,
      "Submap query service to connect to.", this, SLOT(Reset()));
  tracking_frame_property_ = new ::rviz_common::properties::StringProperty(
      "Tracking frame", kDefaultTrackingFrame,
      "Tracking frame, used for fading out submaps.", this);
  slice_high_resolution_enabled_ = new ::rviz_common::properties::BoolProperty(
      "High Resolution", true, "Display high resolution slices.", this,
      SLOT(ResolutionToggled()), this);
  slice_low_resolution_enabled_ = new ::rviz_common::properties::BoolProperty(
      "Low Resolution", false, "Display low resolution slices.", this,
      SLOT(ResolutionToggled()), this);

  callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  callback_group_executor_->add_callback_group(callback_group_, this->get_node_base_interface());
  client_ = this->create_client<::cartographer_ros_msgs::srv::SubmapQuery>(
        kDefaultSubmapQueryServiceName,
        rmw_qos_profile_services_default,
        callback_group_
        );
  trajectories_category_ = new ::rviz_common::properties::Property(
      "Submaps", QVariant(), "List of all submaps, organized by trajectories.",
      this);
  visibility_all_enabled_ = new ::rviz_common::properties::BoolProperty(
      "All", true,
      "Whether submaps from all trajectories should be displayed or not.",
      trajectories_category_, SLOT(AllEnabledToggled()), this);
  pose_markers_all_enabled_ = new ::rviz_common::properties::BoolProperty(
      "All Submap Pose Markers", true,
      "Whether submap pose markers should be displayed or not.",
      trajectories_category_, SLOT(PoseMarkersEnabledToggled()), this);
  fade_out_start_distance_in_meters_ =
      new ::rviz_common::properties::FloatProperty("Fade-out distance", 1.f,
                                "Distance in meters in z-direction beyond "
                                "which submaps will start to fade out.",
                                this);
  const std::string package_path = ament_index_cpp::get_package_share_directory("cartographer_rviz");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory, "FileSystem", "cartographer_rviz");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kGlsl120Directory, "FileSystem",
      "cartographer_rviz");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kScriptsDirectory, "FileSystem",
      "cartographer_rviz");
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

SubmapsDisplay::~SubmapsDisplay() {
  client_.reset();
  trajectories_.clear();
  scene_manager_->destroySceneNode(map_node_);
}

void SubmapsDisplay::Reset() { reset(); }

void SubmapsDisplay::CreateClient() {
  client_ = this->create_client<::cartographer_ros_msgs::srv::SubmapQuery>(
      submap_query_service_property_->getStdString(),
      rmw_qos_profile_services_default,
      callback_group_
      );
}

void SubmapsDisplay::onInitialize() {
  MFDClass::onInitialize();
  map_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  CreateClient();
}

void SubmapsDisplay::reset() {
  MFDClass::reset();
  absl::MutexLock locker(&mutex_);
  client_.reset();
  trajectories_.clear();
  CreateClient();
}

void SubmapsDisplay::processMessage( const ::cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr msg) {
  absl::MutexLock locker(&mutex_);
  map_frame_ = absl::make_unique<std::string>(msg->header.frame_id);
  // In case Cartographer node is relaunched, destroy trajectories from the
  // previous instance.
  for (const ::cartographer_ros_msgs::msg::SubmapEntry& submap_entry : msg->submap) {
    const size_t trajectory_id = submap_entry.trajectory_id;
    if (trajectories_.count(trajectory_id) == 0) {
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
  std::set<int> listed_trajectories;
  for (const ::cartographer_ros_msgs::msg::SubmapEntry& submap_entry : msg->submap) {
    const SubmapId id{submap_entry.trajectory_id, submap_entry.submap_index};
    listed_submaps.insert(id);
    listed_trajectories.insert(submap_entry.trajectory_id);
    if (trajectories_.count(id.trajectory_id) == 0) {
      trajectories_.insert(std::make_pair(
          id.trajectory_id,
          absl::make_unique<Trajectory>(
              absl::make_unique<::rviz_common::properties::BoolProperty>(
                  QString("Trajectory %1").arg(id.trajectory_id),
                  visibility_all_enabled_->getBool(),
                  QString(
                      "List of all submaps in Trajectory %1. The checkbox "
                      "controls whether all submaps in this trajectory should "
                      "be displayed or not.")
                      .arg(id.trajectory_id),
                  trajectories_category_),
              pose_markers_all_enabled_->getBool())));
    }
    auto& trajectory_visibility = trajectories_[id.trajectory_id]->visibility;
    auto& trajectory_submaps = trajectories_[id.trajectory_id]->submaps;
    auto& pose_markers_visibility =
        trajectories_[id.trajectory_id]->pose_markers_visibility;
    if (trajectory_submaps.count(id.submap_index) == 0) {
      // TODO(ojura): Add RViz properties for adjusting submap pose axes
      constexpr float kSubmapPoseAxesLength = 0.3f;
      constexpr float kSubmapPoseAxesRadius = 0.06f;
      trajectory_submaps.emplace(
          id.submap_index,
          absl::make_unique<DrawableSubmap>(
              id, context_, map_node_, trajectory_visibility.get(),
              trajectory_visibility->getBool(),
              pose_markers_visibility->getBool(), kSubmapPoseAxesLength,
              kSubmapPoseAxesRadius));
      trajectory_submaps.at(id.submap_index)
          ->SetSliceVisibility(0, slice_high_resolution_enabled_->getBool());
      trajectory_submaps.at(id.submap_index)
          ->SetSliceVisibility(1, slice_low_resolution_enabled_->getBool());
    }
    trajectory_submaps.at(id.submap_index)->Update(msg->header, submap_entry);
  }
  // Remove all deleted trajectories not mentioned in the SubmapList.
  for (auto it = trajectories_.begin(); it != trajectories_.end();) {
    if (listed_trajectories.count(it->first) == 0) {
      it = trajectories_.erase(it);
    } else {
      ++it;
    }
  }
  // Remove all submaps not mentioned in the SubmapList.
  for (const auto& trajectory_by_id : trajectories_) {
    const int trajectory_id = trajectory_by_id.first;
    auto& trajectory_submaps = trajectory_by_id.second->submaps;
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

void SubmapsDisplay::update(const float , const float) {
  absl::MutexLock locker(&mutex_);
  // Schedule fetching of new submap textures.
  for (const auto& trajectory_by_id : trajectories_) {
    int num_ongoing_requests = 0;
    for (const auto& submap_entry : trajectory_by_id.second->submaps) {
      if (submap_entry.second->QueryInProgress()) {
        ++num_ongoing_requests;
      }
    }
    for (auto it = trajectory_by_id.second->submaps.rbegin();
         it != trajectory_by_id.second->submaps.rend() &&
         num_ongoing_requests < kMaxOnGoingRequestsPerTrajectory;
         ++it) {
      if (it->second->MaybeFetchTexture(client_, callback_group_executor_)) {
        ++num_ongoing_requests;
      }
    }
  }
  if (map_frame_ == nullptr) {
    return;
  }

  // Update the fading by z distance.
  const auto klatest = this->get_clock()->now();
  try {
    const ::geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(
            *map_frame_, tracking_frame_property_->getStdString(), tf2::TimePointZero,std::chrono::milliseconds(100));
    for (auto& trajectory_by_id : trajectories_) {
      for (auto& submap_entry : trajectory_by_id.second->submaps) {
        submap_entry.second->SetAlpha(
            transform_stamped.transform.translation.z,
            fade_out_start_distance_in_meters_->getFloat());
      }
    }
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not compute submap fading: %s", ex.what());
  }
  // Update the map frame to fixed frame transform.
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (context_->getFrameManager()->getTransform(*map_frame_, klatest, position,
                                                orientation)) {
    map_node_->setPosition(position);
    map_node_->setOrientation(orientation);
    context_->queueRender();
  }
}

void SubmapsDisplay::AllEnabledToggled() {
  absl::MutexLock locker(&mutex_);
  const bool visible = visibility_all_enabled_->getBool();
  for (auto& trajectory_by_id : trajectories_) {
    trajectory_by_id.second->visibility->setBool(visible);
  }
}

void SubmapsDisplay::PoseMarkersEnabledToggled() {
  absl::MutexLock locker(&mutex_);
  const bool visible = pose_markers_all_enabled_->getBool();
  for (auto& trajectory_by_id : trajectories_) {
    trajectory_by_id.second->pose_markers_visibility->setBool(visible);
  }
}

void SubmapsDisplay::ResolutionToggled() {
  absl::MutexLock locker(&mutex_);
  for (auto& trajectory_by_id : trajectories_) {
    for (auto& submap_entry : trajectory_by_id.second->submaps) {
      submap_entry.second->SetSliceVisibility(
          0, slice_high_resolution_enabled_->getBool());
      submap_entry.second->SetSliceVisibility(
          1, slice_low_resolution_enabled_->getBool());
    }
  }
}

void Trajectory::AllEnabledToggled() {
  const bool visible = visibility->getBool();
  for (auto& submap_entry : submaps) {
    submap_entry.second->set_visibility(visible);
  }
}

void Trajectory::PoseMarkersEnabledToggled() {
  const bool visible = pose_markers_visibility->getBool();
  for (auto& submap_entry : submaps) {
    submap_entry.second->set_pose_markers_visibility(visible);
  }
}

Trajectory::Trajectory(std::unique_ptr<::rviz_common::properties::BoolProperty> property,
                       const bool pose_markers_enabled)
    : visibility(std::move(property)) {
  ::QObject::connect(visibility.get(), SIGNAL(changed()), this,
                     SLOT(AllEnabledToggled()));
  // Add toggle for submap pose markers as the first entry of the visibility
  // property list of this trajectory.
  pose_markers_visibility = absl::make_unique<::rviz_common::properties::BoolProperty>(
      QString("Submap Pose Markers"), pose_markers_enabled,
      QString("Toggles the submap pose markers of this trajectory."),
      visibility.get());
  ::QObject::connect(pose_markers_visibility.get(), SIGNAL(changed()), this,
                     SLOT(PoseMarkersEnabledToggled()));
}

}  // namespace cartographer_rviz

PLUGINLIB_EXPORT_CLASS(cartographer_rviz::SubmapsDisplay, rviz_common::Display)
