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

#include "submaps_display.h"

#include <OgreResourceGroupManager.h>
#include <OgreSceneManager.h>
#include <cartographer/common/mutex.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <cartographer_ros_msgs/SubmapQuery.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>

#include "node_constants.h"

namespace cartographer_ros {
namespace rviz {

namespace {

constexpr int kMaxOnGoingRequests = 6;
constexpr char kMaterialsDirectory[] = "/ogre_media/materials";
constexpr char kGlsl120Directory[] = "/glsl120";
constexpr char kScriptsDirectory[] = "/scripts";
constexpr char kDefaultMapFrame[] = "map";
constexpr char kDefaultTrackingFrame[] = "base_link";

}  // namespace

SubmapsDisplay::SubmapsDisplay()
    : Display(),
      scene_manager_listener_([this]() { UpdateMapTexture(); }),
      tf_listener_(tf_buffer_) {
  connect(this, SIGNAL(SubmapListUpdated()), this, SLOT(RequestNewSubmaps()));
  topic_property_ = new ::rviz::RosTopicProperty(
      "Topic", QString("/cartographer/") + kSubmapListTopic,
      QString::fromStdString(
          ros::message_traits::datatype<::cartographer_ros_msgs::SubmapList>()),
      "cartographer_ros_msgs::SubmapList topic to subscribe to.", this,
      SLOT(UpdateSubmapTopicOrService()));
  submap_query_service_property_ = new ::rviz::StringProperty(
      "Submap query service",
      QString("/cartographer/") + kSubmapQueryServiceName,
      "Submap query service to connect to.", this,
      SLOT(UpdateSubmapTopicOrService()));
  map_frame_property_ = new ::rviz::StringProperty(
      "Map frame", kDefaultMapFrame, "Map frame, used for fading out submaps.",
      this);
  tracking_frame_property_ = new ::rviz::StringProperty(
      "Tracking frame", kDefaultTrackingFrame,
      "Tracking frame, used for fading out submaps.", this);
  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>("");
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

SubmapsDisplay::~SubmapsDisplay() { UnsubscribeAndClear(); }

void SubmapsDisplay::onInitialize() {
  scene_manager_->addListener(&scene_manager_listener_);
  UpdateSubmapTopicOrService();
}

void SubmapsDisplay::UpdateSubmapTopicOrService() {
  UnsubscribeAndClear();
  Subscribe();
}

void SubmapsDisplay::reset() {
  Display::reset();
  UpdateSubmapTopicOrService();
}

void SubmapsDisplay::onEnable() { Subscribe(); }

void SubmapsDisplay::onDisable() { UnsubscribeAndClear(); }

void SubmapsDisplay::Subscribe() {
  if (!isEnabled()) {
    return;
  }

  client_ = update_nh_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
      submap_query_service_property_->getStdString());

  if (!topic_property_->getTopic().isEmpty()) {
    try {
      submap_list_subscriber_ =
          update_nh_.subscribe(topic_property_->getTopicStd(), 1,
                               &SubmapsDisplay::IncomingSubmapList, this,
                               ros::TransportHints().reliable());
      setStatus(::rviz::StatusProperty::Ok, "Topic", "OK");
    } catch (const ros::Exception& ex) {
      setStatus(::rviz::StatusProperty::Error, "Topic",
                QString("Error subscribing: ") + ex.what());
    }
  }
}

void SubmapsDisplay::IncomingSubmapList(
    const ::cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  submap_list_ = *msg;
  Q_EMIT SubmapListUpdated();
}

void SubmapsDisplay::UnsubscribeAndClear() {
  submap_list_subscriber_.shutdown();
  client_.shutdown();
  ::cartographer::common::MutexLocker locker(&mutex_);
  trajectories_.clear();
}

void SubmapsDisplay::RequestNewSubmaps() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  for (int trajectory_id = 0; trajectory_id < submap_list_.trajectory.size();
       ++trajectory_id) {
    if (trajectory_id >= trajectories_.size()) {
      trajectories_.emplace_back(new Trajectory);
    }
    const std::vector<::cartographer_ros_msgs::SubmapEntry>& submap_entries =
        submap_list_.trajectory[trajectory_id].submap;
    if (submap_entries.empty()) {
      return;
    }
    for (int submap_id = trajectories_[trajectory_id]->Size();
         submap_id < submap_entries.size(); ++submap_id) {
      trajectories_[trajectory_id]->Add(submap_id, trajectory_id,
                                        context_->getFrameManager(),
                                        context_->getSceneManager());
    }
  }
  int num_ongoing_requests = 0;
  for (const auto& trajectory : trajectories_) {
    for (int i = 0; i < trajectory->Size(); ++i) {
      if (trajectory->Get(i).QueryInProgress()) {
        ++num_ongoing_requests;
        if (num_ongoing_requests == kMaxOnGoingRequests) {
          return;
        }
      }
    }
  }
  for (int trajectory_id = 0; trajectory_id < submap_list_.trajectory.size();
       ++trajectory_id) {
    const std::vector<::cartographer_ros_msgs::SubmapEntry>& submap_entries =
        submap_list_.trajectory[trajectory_id].submap;
    for (int submap_id = submap_entries.size() - 1; submap_id >= 0;
         --submap_id) {
      if (trajectories_[trajectory_id]->Get(submap_id).Update(
              submap_entries[submap_id], &client_)) {
        ++num_ongoing_requests;
        if (num_ongoing_requests == kMaxOnGoingRequests) {
          return;
        }
      }
    }
  }
}

void SubmapsDisplay::UpdateMapTexture() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  for (auto& trajectory : trajectories_) {
    for (int i = 0; i < trajectory->Size(); ++i) {
      trajectory->Get(i).Transform(ros::Time(0) /* latest */);
      try {
        const ::geometry_msgs::TransformStamped transform_stamped =
            tf_buffer_.lookupTransform(map_frame_property_->getStdString(),
                                       tracking_frame_property_->getStdString(),
                                       ros::Time(0) /* latest */);
        trajectory->Get(i).SetAlpha(
            transform_stamped.transform.translation.z);
      } catch (const tf2::TransformException& ex) {
        ROS_WARN("Could not compute submap fading: %s", ex.what());
      }
    }
  }
}

}  // namespace rviz
}  // namespace cartographer_ros

PLUGINLIB_EXPORT_CLASS(cartographer_ros::rviz::SubmapsDisplay, ::rviz::Display)
