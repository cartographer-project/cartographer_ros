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

#ifndef CARTOGRAPHER_RVIZ_SRC_DRAWABLE_SUBMAP_H_
#define CARTOGRAPHER_RVIZ_SRC_DRAWABLE_SUBMAP_H_

#include <future>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "OgreManualObject.h"
#include "OgreMaterial.h"
#include "OgreQuaternion.h"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"
#include "OgreTexture.h"
#include "OgreVector3.h"
#include "cartographer/common/mutex.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "ros/ros.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"

namespace cartographer_rviz {

// Contains all the information needed to render a submap onto the final
// texture representing the whole map.
class DrawableSubmap : public QObject {
  Q_OBJECT

 public:
  // Each submap is identified by a 'trajectory_id' plus a 'submap_index'.
  // 'scene_manager' is the Ogre scene manager to which to add a node.
  DrawableSubmap(int trajectory_id, int submap_index,
                 Ogre::SceneManager* scene_manager);
  ~DrawableSubmap() override;
  DrawableSubmap(const DrawableSubmap&) = delete;
  DrawableSubmap& operator=(const DrawableSubmap&) = delete;

  // Updates the 'metadata' for this submap. If necessary, the next call to
  // MaybeFetchTexture() will fetch a new submap texture.
  void Update(const ::std_msgs::Header& header,
              const ::cartographer_ros_msgs::SubmapEntry& metadata,
              ::rviz::FrameManager* frame_manager);

  // If an update is needed, it will send an RPC using 'client' to request the
  // new data for the submap and returns true.
  bool MaybeFetchTexture(ros::ServiceClient* client);

  // Returns whether an RPC is in progress.
  bool QueryInProgress();

  // Sets the alpha of the submap taking into account its slice height and the
  // 'current_tracking_z'.
  void SetAlpha(double current_tracking_z);

 Q_SIGNALS:
  // RPC request succeeded.
  void RequestSucceeded();

 private Q_SLOTS:
  // Callback when an rpc request succeeded.
  void UpdateSceneNode();

 private:
  void UpdateTransform();
  float UpdateAlpha(float target_alpha);

  const int trajectory_id_;
  const int submap_index_;

  ::cartographer::common::Mutex mutex_;
  Ogre::SceneManager* const scene_manager_;
  Ogre::SceneNode* const scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  double submap_z_ = 0. GUARDED_BY(mutex_);
  Ogre::Vector3 position_ GUARDED_BY(mutex_);
  Ogre::Quaternion orientation_ GUARDED_BY(mutex_);
  Eigen::Affine3d slice_pose_ GUARDED_BY(mutex_);
  std::chrono::milliseconds last_query_timestamp_ GUARDED_BY(mutex_);
  bool query_in_progress_ = false GUARDED_BY(mutex_);
  int metadata_version_ = -1 GUARDED_BY(mutex_);
  int texture_version_ = -1 GUARDED_BY(mutex_);
  std::future<void> rpc_request_future_;
  ::cartographer_ros_msgs::SubmapQuery::Response response_ GUARDED_BY(mutex_);
  float current_alpha_ = 0.f;
};

}  // namespace cartographer_rviz

#endif  // CARTOGRAPHER_RVIZ_SRC_DRAWABLE_SUBMAP_H_
