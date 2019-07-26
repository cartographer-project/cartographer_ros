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

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "gflags/gflags.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class OccupancyGridNode : public rclcpp::Node
{
 public:
  explicit OccupancyGridNode(const double resolution, const double publish_period_sec)
  : Node("cartographer_occupancy_grid_node"),
    resolution_(resolution)
  {
    client_ = this->create_client<cartographer_ros_msgs::srv::SubmapQuery>(kSubmapQueryServiceName);

    occupancy_grid_publisher_ = this->create_publisher<::nav_msgs::msg::OccupancyGrid>(
        kOccupancyGridTopic, rclcpp::QoS(10).transient_local());

    occupancy_grid_publisher_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(publish_period_sec * 1000)),
      [this]() {
        DrawAndPublish();
      });

    auto handleSubmapList =
      [this](const typename cartographer_ros_msgs::msg::SubmapList::SharedPtr msg) -> void
      {
        ::cartographer::common::MutexLocker locker(&mutex_);

          // We do not do any work if nobody listens.
          if (this->count_publishers(kSubmapListTopic) == 0){
            RCLCPP_WARN(this->get_logger(), "topic(/submap_list) is not found");
            return;
          }

          // Keep track of submap IDs that don't appear in the message anymore.
          std::set<SubmapId> submap_ids_to_delete;
          for (const auto& pair : submap_slices_) {
            submap_ids_to_delete.insert(pair.first);
          }

          for (const auto& submap_msg : msg->submap) {
            const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
            submap_ids_to_delete.erase(id);
            SubmapSlice& submap_slice = submap_slices_[id];
            submap_slice.pose = ToRigid3d(submap_msg.pose);
            submap_slice.metadata_version = submap_msg.submap_version;
            if (submap_slice.surface != nullptr &&
                submap_slice.version == submap_msg.submap_version) {
              continue;
            }

            while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
              return;
              }
              RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }

            auto srv_request = std::make_shared<cartographer_ros_msgs::srv::SubmapQuery::Request>();
            srv_request->trajectory_id = id.trajectory_id;
            srv_request->submap_index = id.submap_index;

            using ServiceResponseFuture =
              ::rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedFuture;
            auto response_received_callback = [&submap_slice](ServiceResponseFuture future) {
              auto fetched_textures = cartographer_ros::FetchSubmapTextures(future.get());
              if (fetched_textures == nullptr) {
                return;
              }
              CHECK(!fetched_textures->textures.empty());
              submap_slice.version = fetched_textures->version;

              // We use the first texture only. By convention this is the highest
              // resolution texture and that is the one we want to use to construct the
              // map for ROS.
              const auto fetched_texture = fetched_textures->textures.begin();
              submap_slice.width = fetched_texture->width;
              submap_slice.height = fetched_texture->height;
              submap_slice.slice_pose = fetched_texture->slice_pose;
              submap_slice.resolution = fetched_texture->resolution;
              submap_slice.cairo_data.clear();
              submap_slice.surface =  ::cartographer::io::DrawTexture(
                  fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
                  fetched_texture->width, fetched_texture->height,
                  &submap_slice.cairo_data);
            };

            auto future_result = client_->async_send_request(srv_request, response_received_callback);
          }

          // Delete all submaps that didn't appear in the message.
          for (const auto& id : submap_ids_to_delete) {
            submap_slices_.erase(id);
          }

          last_timestamp_ = msg->header.stamp;
          last_frame_id_ = msg->header.frame_id;
      };

    submap_list_subscriber_ = create_subscription<cartographer_ros_msgs::msg::SubmapList>(
      kSubmapListTopic, rclcpp::QoS(10), handleSubmapList);
  }

  void DrawAndPublish(void)
  {
    if (submap_slices_.empty() || last_frame_id_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "submap_slices and last_frame_id is empty");
      return;
    }

    ::cartographer::common::MutexLocker locker(&mutex_);
    auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
        painted_slices, resolution_, last_frame_id_, last_timestamp_);

    occupancy_grid_publisher_->publish(*msg_ptr);
  }

 private:
  const double resolution_;

  ::cartographer::common::Mutex mutex_;
  ::rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client_ GUARDED_BY(mutex_);
  ::rclcpp::Subscription<cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_subscriber_ GUARDED_BY(mutex_);
  ::rclcpp::Publisher<::nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_ GUARDED_BY(mutex_);

  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::rclcpp::TimerBase::SharedPtr occupancy_grid_publisher_timer_;

  std::string last_frame_id_;
  ::rclcpp::Time last_timestamp_;
};

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::rclcpp::init(argc, argv);

  auto node = std::make_shared<cartographer_ros::OccupancyGridNode>(FLAGS_resolution, FLAGS_publish_period_sec);

  rclcpp::spin(node);
  ::rclcpp::shutdown();
  return 0;
}
