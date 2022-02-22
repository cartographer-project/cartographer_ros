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

#ifndef CARTOGRAPHER_RVIZ_SRC_SUBMAPS_DISPLAY_H_
#define CARTOGRAPHER_RVIZ_SRC_SUBMAPS_DISPLAY_H_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <dlfcn.h>
#include "absl/synchronization/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_rviz/drawable_submap.h"
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace cartographer_rviz {

// TODO(gaschler): This should be a private class in SubmapsDisplay,
// unfortunately, QT does not allow for this. Move the logic out of the struct
// and use just one slot for all changes.
struct Trajectory : public QObject {
  Q_OBJECT

 public:
  Trajectory(std::unique_ptr<::rviz_common::properties::BoolProperty> property,
             bool pose_markers_enabled);

  std::unique_ptr<::rviz_common::properties::BoolProperty> visibility;
  std::unique_ptr<::rviz_common::properties::BoolProperty> pose_markers_visibility;
  std::map<int, std::unique_ptr<DrawableSubmap>> submaps;

 private Q_SLOTS:
  void AllEnabledToggled();
  void PoseMarkersEnabledToggled();
};

// RViz plugin used for displaying maps which are represented by a collection of
// submaps.
//
// We show an X-ray view of the map which is achieved by shipping textures for
// every submap containing pre-multiplied alpha and grayscale values, these are
// then alpha blended together.
class SubmapsDisplay
    : public ::rviz_common::MessageFilterDisplay<::cartographer_ros_msgs::msg::SubmapList>, rclcpp::Node {
  Q_OBJECT

 public:
  SubmapsDisplay();
  ~SubmapsDisplay() override;



  SubmapsDisplay(const SubmapsDisplay&) = delete;
  SubmapsDisplay& operator=(const SubmapsDisplay&) = delete;

 private Q_SLOTS:
  void Reset();
  void AllEnabledToggled();
  void PoseMarkersEnabledToggled();
  void ResolutionToggled();

 private:
  void CreateClient();

  // These are called by RViz and therefore do not adhere to the style guide.
  void onInitialize() override;
  void reset() override;
  void processMessage(
        const ::cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr msg) override;
  void update(float , float) override;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client_;
  ::rviz_common::properties::StringProperty* submap_query_service_property_;
  std::unique_ptr<std::string> map_frame_;
  ::rviz_common::properties::StringProperty* tracking_frame_property_;
  Ogre::SceneNode* map_node_ = nullptr;  // Represents the map frame.
  std::map<int, std::unique_ptr<Trajectory>> trajectories_ GUARDED_BY(mutex_);
  absl::Mutex mutex_;
  ::rviz_common::properties::BoolProperty* slice_high_resolution_enabled_;
  ::rviz_common::properties::BoolProperty* slice_low_resolution_enabled_;
  ::rviz_common::properties::Property* trajectories_category_;
  ::rviz_common::properties::BoolProperty* visibility_all_enabled_;
  ::rviz_common::properties::BoolProperty* pose_markers_all_enabled_;
  ::rviz_common::properties::FloatProperty* fade_out_start_distance_in_meters_;

};

}  // namespace cartographer_rviz

#endif  // CARTOGRAPHER_RVIZ_SRC_SUBMAPS_DISPLAY_H_
