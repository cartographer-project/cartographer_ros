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

#include <cartographer/common/mutex.h>
#include <cartographer/common/port.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <rviz/message_filter_display.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

#include "drawable_submap.h"

namespace cartographer_rviz {

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
  void CreateClient();

  void onInitialize() override;
  void reset() override;
  void processMessage(
      const ::cartographer_ros_msgs::SubmapList::ConstPtr& msg) override;
  void update(float wall_dt, float ros_dt) override;

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

}  // namespace cartographer_rviz

#endif  // CARTOGRAPHER_RVIZ_SRC_SUBMAPS_DISPLAY_H_
