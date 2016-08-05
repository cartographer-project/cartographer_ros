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

#ifndef CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_SUBMAPS_DISPLAY_H_
#define CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_SUBMAPS_DISPLAY_H_

#include <OgreMaterial.h>
#include <OgreSceneManager.h>
#include <OgreSharedPtr.h>
#include <OgreTexture.h>
#include <OgreVector3.h>
#include <cartographer/common/mutex.h>
#include <cartographer/common/port.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <nav_msgs/MapMetaData.h>
#include <ros/time.h>
#include <rviz/message_filter_display.h>
#include <tf/tfMessage.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

#include "drawable_submap.h"

namespace cartographer_ros {
namespace rviz {

// RViz plugin used for displaying maps which are represented by a collection of
// submaps.
//
// We show an X-ray view of the map which is achieved by shipping textures for
// every submap containing pre-multiplied alpha and grayscale values, these are
// then alpha blended together.
class SubmapsDisplay
    : public ::rviz::MessageFilterDisplay<::cartographer_ros_msgs::SubmapList> {
  Q_OBJECT

 public:
  SubmapsDisplay();
  ~SubmapsDisplay() override;

  SubmapsDisplay(const SubmapsDisplay&) = delete;
  SubmapsDisplay& operator=(const SubmapsDisplay&) = delete;

 private Q_SLOTS:
  void Reset();

 private:
  class SceneManagerListener : public Ogre::SceneManager::Listener {
   public:
    SceneManagerListener(std::function<void()> callback)
        : callback_(callback) {}
    void preUpdateSceneGraph(Ogre::SceneManager* source, Ogre::Camera* camera) {
      callback_();
    }

   private:
    std::function<void()> callback_;
  };

  void CreateClient();

  void onInitialize() override;
  void reset() override;
  void processMessage(
      const ::cartographer_ros_msgs::SubmapList::ConstPtr& msg) override;
  void update(float wall_dt, float ros_dt) override;

  void UpdateTransforms();

  SceneManagerListener scene_manager_listener_;
  ::tf2_ros::Buffer tf_buffer_;
  ::tf2_ros::TransformListener tf_listener_;
  ros::ServiceClient client_;
  ::rviz::StringProperty* submap_query_service_property_;
  ::rviz::StringProperty* map_frame_property_;
  ::rviz::StringProperty* tracking_frame_property_;
  using Trajectory = std::vector<std::unique_ptr<DrawableSubmap>>;
  std::vector<Trajectory> trajectories_ GUARDED_BY(mutex_);
  ::cartographer::common::Mutex mutex_;
};

}  // namespace rviz
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_GOOGLE_CARTOGRAPHER_SRC_SUBMAPS_DISPLAY_H_
