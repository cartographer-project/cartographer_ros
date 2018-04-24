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
#include <memory>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "OgreSceneManager.h"
#include "OgreSceneNode.h"
#include "cartographer/common/mutex.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_rviz/ogre_slice.h"
#include "ros/ros.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/movable_text.h"
#include "rviz/properties/bool_property.h"

namespace cartographer_rviz {

// Contains all the information needed to render a submap onto the final
// texture representing the whole map.
class DrawableSubmap : public QObject {
  Q_OBJECT

 public:
  DrawableSubmap(const ::cartographer::mapping::SubmapId& submap_id,
                 ::rviz::DisplayContext* display_context,
                 Ogre::SceneNode* map_node, ::rviz::Property* submap_category,
                 bool visible, float pose_axes_length, float pose_axes_radius);
  ~DrawableSubmap() override;
  DrawableSubmap(const DrawableSubmap&) = delete;
  DrawableSubmap& operator=(const DrawableSubmap&) = delete;

  // Updates the 'metadata' for this submap. If necessary, the next call to
  // MaybeFetchTexture() will fetch a new submap texture.
  void Update(const ::std_msgs::Header& header,
              const ::cartographer_ros_msgs::SubmapEntry& metadata);

  // If an update is needed, it will send an RPC using 'client' to request the
  // new data for the submap and returns true.
  bool MaybeFetchTexture(ros::ServiceClient* client);

  // Returns whether an RPC is in progress.
  bool QueryInProgress();

  // Sets the alpha of the submap taking into account its slice height and the
  // 'current_tracking_z'. 'fade_out_start_distance_in_meters' defines the
  // distance in z direction in meters, before which the submap will be shown
  // at full opacity.
  void SetAlpha(double current_tracking_z, float fade_out_distance_in_meters);

  // Sets the visibility of a slice. It will be drawn if the parent submap
  // is also visible.
  void SetSliceVisibility(size_t slice_index, bool visible);

  ::cartographer::mapping::SubmapId id() const { return id_; }
  int version() const { return metadata_version_; }
  bool visibility() const { return visibility_->getBool(); }
  void set_visibility(const bool visibility) {
    visibility_->setBool(visibility);
  }

 Q_SIGNALS:
  // RPC request succeeded.
  void RequestSucceeded();

 private Q_SLOTS:
  // Callback when an rpc request succeeded.
  void UpdateSceneNode();
  void ToggleVisibility();

 private:
  const ::cartographer::mapping::SubmapId id_;

  ::cartographer::common::Mutex mutex_;
  ::rviz::DisplayContext* const display_context_;
  Ogre::SceneNode* const submap_node_;
  Ogre::SceneNode* const submap_id_text_node_;
  std::vector<std::unique_ptr<OgreSlice>> ogre_slices_;
  ::cartographer::transform::Rigid3d pose_ GUARDED_BY(mutex_);
  ::rviz::Axes pose_axes_;
  ::rviz::MovableText submap_id_text_;
  std::chrono::milliseconds last_query_timestamp_ GUARDED_BY(mutex_);
  bool query_in_progress_ GUARDED_BY(mutex_) = false;
  int metadata_version_ GUARDED_BY(mutex_) = -1;
  std::future<void> rpc_request_future_;
  std::unique_ptr<::cartographer::io::SubmapTextures> submap_textures_
      GUARDED_BY(mutex_);
  float current_alpha_ = 0.f;
  std::unique_ptr<::rviz::BoolProperty> visibility_;
};

}  // namespace cartographer_rviz

#endif  // CARTOGRAPHER_RVIZ_SRC_DRAWABLE_SUBMAP_H_
