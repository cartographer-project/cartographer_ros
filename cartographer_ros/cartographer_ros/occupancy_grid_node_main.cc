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
#include "cartographer/common/make_unique.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");

namespace cartographer_ros {
namespace {

using ::cartographer::mapping::SubmapId;

constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

Eigen::Affine3d ToEigen(const ::cartographer::transform::Rigid3d& rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}

struct SubmapState {
  SubmapState(const ::cartographer::mapping::SubmapId& id) {}
  ~SubmapState() {
    if (texture.surface != nullptr) {
      cairo_surface_destroy(texture.surface);
    }
  }

  struct {
    int width;
    int height;
    int version;
    double resolution;
    ::cartographer::transform::Rigid3d slice_pose;
    cairo_surface_t* surface = nullptr;

    // Pixel data used by 'surface'. Must outlive 'surface'.
    std::vector<uint32_t> cairo_data;
  } texture;
  ::cartographer::transform::Rigid3d pose;
  int metadata_version = -1;
};

void CairoDrawEachSubmap(const double scale,
                         std::map<SubmapId, SubmapState>* submaps, cairo_t* cr,
                         std::function<void(const SubmapState&)> func) {
  cairo_scale(cr, scale, scale);

  for (auto& pair : *submaps) {
    auto& submap_state = pair.second;
    if (submap_state.texture.surface == nullptr) {
      return;
    }
    const Eigen::Matrix4d homo =
        ToEigen(submap_state.pose * submap_state.texture.slice_pose).matrix();

    cairo_save(cr);
    cairo_matrix_t matrix;
    cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                      homo(0, 3), -homo(1, 3));
    cairo_transform(cr, &matrix);

    const double submap_resolution = submap_state.texture.resolution;
    cairo_scale(cr, submap_resolution, submap_resolution);

    func(submap_state);

    cairo_restore(cr);
  }
}

class Node {
 public:
  explicit Node(double resolution);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const string& frame_id, const ros::Time& time);
  void PublishOccupancyGrid(const string& frame_id, const ros::Time& time,
                            const Eigen::Array2f& origin,
                            const Eigen::Array2i& size,
                            cairo_surface_t* surface);

  ::ros::NodeHandle node_handle_;
  ::cartographer::common::Mutex mutex_;
  const double resolution_;
  ::ros::ServiceClient client_;
  ::ros::Subscriber submap_list_subscriber_;
  ::ros::Publisher occupancy_grid_publisher_;
  std::map<SubmapId, SubmapState> submaps_;
};

Node::Node(const double resolution)
    : resolution_(resolution),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
              true /* latched */))

{}

void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  ::cartographer::common::MutexLocker locker(&mutex_);

  // We do not do any work if nobody listens.
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }
  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    if (submaps_.count(id) == 0) {
      submaps_.emplace(std::piecewise_construct, std::forward_as_tuple(id),
                       std::forward_as_tuple(id));
    }
    SubmapState& submap_state = submaps_.at(id);
    submap_state.pose = ToRigid3d(submap_msg.pose);
    submap_state.metadata_version = submap_msg.submap_version;
    if (submap_state.texture.surface != nullptr &&
        submap_state.texture.version == submap_msg.submap_version) {
      continue;
    }

    auto fetched_texture = ::cartographer_ros::FetchSubmapTexture(id, &client_);
    if (fetched_texture == nullptr) {
      continue;
    }
    submap_state.texture.width = fetched_texture->width;
    submap_state.texture.height = fetched_texture->height;
    submap_state.texture.version = fetched_texture->version;
    submap_state.texture.slice_pose = fetched_texture->slice_pose;
    submap_state.texture.resolution = fetched_texture->resolution;

    // Properly dealing with a non-common stride would make this code much more
    // complicated. Let's check that it is not needed.
    const int expected_stride = 4 * submap_state.texture.width;
    CHECK_EQ(expected_stride, cairo_format_stride_for_width(
                                  kCairoFormat, submap_state.texture.width));
    submap_state.texture.cairo_data.clear();
    if (submap_state.texture.surface != nullptr) {
      cairo_surface_destroy(submap_state.texture.surface);
    }
    for (size_t i = 0; i < fetched_texture->intensity.size(); ++i) {
      const uint8_t intensity = fetched_texture->intensity.at(i);
      const uint8_t alpha = fetched_texture->alpha.at(i);
      submap_state.texture.cairo_data.push_back(
          (alpha << 24) | (intensity << 16) | (intensity << 8) | intensity);
    }

    submap_state.texture.surface = cairo_image_surface_create_for_data(
        reinterpret_cast<unsigned char*>(
            submap_state.texture.cairo_data.data()),
        kCairoFormat, submap_state.texture.width, submap_state.texture.height,
        expected_stride);
    CHECK_EQ(cairo_surface_status(submap_state.texture.surface),
             CAIRO_STATUS_SUCCESS)
        << cairo_status_to_string(
               cairo_surface_status(submap_state.texture.surface));
  }
  DrawAndPublish(msg->header.frame_id, msg->header.stamp);
}

void Node::DrawAndPublish(const string& frame_id, const ros::Time& time) {
  if (submaps_.empty()) {
    return;
  }

  Eigen::AlignedBox2f bounding_box;
  {
    cairo_surface_t* surface = cairo_image_surface_create(kCairoFormat, 1, 1);
    cairo_t* cr = cairo_create(surface);
    const auto update_bounding_box = [&bounding_box](cairo_t* cr, double x,
                                                     double y) {
      cairo_user_to_device(cr, &x, &y);
      bounding_box.extend(Eigen::Vector2f(x, y));
    };

    CairoDrawEachSubmap(
        1. / resolution_, &submaps_, cr,
        [&update_bounding_box, &bounding_box,
         cr](const SubmapState& submap_state) {
          update_bounding_box(cr, 0, 0);
          update_bounding_box(cr, submap_state.texture.width, 0);
          update_bounding_box(cr, 0, submap_state.texture.height);
          update_bounding_box(cr, submap_state.texture.width,
                              submap_state.texture.height);

        });
    cairo_surface_destroy(surface);
    cairo_destroy(cr);
  }

  const int kPaddingPixel = 5;
  const Eigen::Array2i size(
      std::ceil(bounding_box.sizes().x()) + 2 * kPaddingPixel,
      std::ceil(bounding_box.sizes().y()) + 2 * kPaddingPixel);
  const Eigen::Array2f origin(-bounding_box.min().x() + kPaddingPixel,
                              -bounding_box.min().y() + kPaddingPixel);

  {
    cairo_surface_t* surface =
        cairo_image_surface_create(kCairoFormat, size.x(), size.y());
    cairo_t* cr = cairo_create(surface);
    // This translates to (128, 128, 128) for unknown values.
    cairo_set_source_rgba(cr, 0.5, 0.5, 0.5, 1.);
    cairo_paint(cr);
    cairo_translate(cr, origin.x(), origin.y());
    CairoDrawEachSubmap(
        1. / resolution_, &submaps_, cr, [cr](const SubmapState& submap_state) {
          cairo_set_source_surface(cr, submap_state.texture.surface, 0., 0.);
          cairo_paint(cr);
        });
    cairo_surface_flush(surface);
    cairo_destroy(cr);
    PublishOccupancyGrid(frame_id, time, origin, size, surface);
    cairo_surface_destroy(surface);
  }
}

void Node::PublishOccupancyGrid(const string& frame_id, const ros::Time& time,
                                const Eigen::Array2f& origin,
                                const Eigen::Array2i& size,
                                cairo_surface_t* surface) {
  nav_msgs::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = time;
  occupancy_grid.header.frame_id = frame_id;
  occupancy_grid.info.map_load_time = occupancy_grid.header.stamp;
  occupancy_grid.info.resolution = resolution_;
  occupancy_grid.info.width = size.x();
  occupancy_grid.info.height = size.y();
  occupancy_grid.info.origin.position.x = -origin.x() * resolution_;
  occupancy_grid.info.origin.position.y =
      (-size.y() + origin.y()) * resolution_;
  occupancy_grid.info.origin.position.z = 0.;
  occupancy_grid.info.origin.orientation.w = 1.;
  occupancy_grid.info.origin.orientation.x = 0.;
  occupancy_grid.info.origin.orientation.y = 0.;
  occupancy_grid.info.origin.orientation.z = 0.;

  unsigned char* data = cairo_image_surface_get_data(surface);
  occupancy_grid.data.reserve(size.x() * size.y());
  for (int y = size.y() - 1; y >= 0; --y) {
    for (int x = 0; x < size.x(); ++x) {
      const unsigned char color = data[y * 4 * size.x() + 4 * x];
      const int value = color == 128 ? -1 : ((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid.data.push_back(value);
    }
  }
  occupancy_grid_publisher_.publish(occupancy_grid);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution);

  ::ros::spin();
  ::ros::shutdown();
}
