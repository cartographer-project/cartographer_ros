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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;
constexpr double kTfBufferCacheTimeInSeconds = 1e6;

// Unique default topic names. Expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";

Node::Node(const NodeOptions& options)
    : options_(options),
      tf_buffer_(::ros::Duration(kTfBufferCacheTimeInSeconds)),
      tf_(tf_buffer_) {}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
}

void Node::Initialize() {
  carto::common::MutexLocker lock(&mutex_);
  std::unordered_set<string> expected_sensor_ids;

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  if (options_.use_laser_scan) {
    horizontal_laser_scan_subscriber_ = node_handle_.subscribe(
        kLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
            [this](const sensor_msgs::LaserScan::ConstPtr& msg) {
              map_builder_bridge_->sensor_bridge()->HandleLaserScanMessage(
                  kLaserScanTopic, msg);
            }));
    expected_sensor_ids.insert(kLaserScanTopic);
  }
  if (options_.use_multi_echo_laser_scan) {
    horizontal_laser_scan_subscriber_ = node_handle_.subscribe(
        kMultiEchoLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::MultiEchoLaserScan::ConstPtr&)>(
            [this](const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
              map_builder_bridge_->sensor_bridge()
                  ->HandleMultiEchoLaserScanMessage(kMultiEchoLaserScanTopic,
                                                    msg);
            }));
    expected_sensor_ids.insert(kMultiEchoLaserScanTopic);
  }

  // For 3D SLAM, subscribe to all point clouds topics.
  if (options_.num_point_clouds > 0) {
    for (int i = 0; i < options_.num_point_clouds; ++i) {
      string topic = kPointCloud2Topic;
      if (options_.num_point_clouds > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      point_cloud_subscribers_.push_back(node_handle_.subscribe(
          topic, kInfiniteSubscriberQueueSize,
          boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
              [this, topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                map_builder_bridge_->sensor_bridge()->HandlePointCloud2Message(
                    topic, msg);
              })));
      expected_sensor_ids.insert(topic);
    }
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (options_.map_builder_options.use_trajectory_builder_3d() ||
      (options_.map_builder_options.use_trajectory_builder_2d() &&
       options_.map_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    imu_subscriber_ = node_handle_.subscribe(
        kImuTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::Imu::ConstPtr& msg)>(
            [this](const sensor_msgs::Imu::ConstPtr& msg) {
              map_builder_bridge_->sensor_bridge()->HandleImuMessage(kImuTopic,
                                                                     msg);
            }));
    expected_sensor_ids.insert(kImuTopic);
  }

  if (options_.use_odometry) {
    odometry_subscriber_ = node_handle_.subscribe(
        kOdometryTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
            [this](const nav_msgs::Odometry::ConstPtr& msg) {
              map_builder_bridge_->sensor_bridge()->HandleOdometryMessage(
                  kOdometryTopic, msg);
            }));
    expected_sensor_ids.insert(kOdometryTopic);
  }

  map_builder_bridge_ = carto::common::make_unique<MapBuilderBridge>(
      options_, expected_sensor_ids, &tf_buffer_);

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  submap_query_server_ = node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this);

  if (options_.map_builder_options.use_trajectory_builder_2d()) {
    occupancy_grid_publisher_ =
        node_handle_.advertise<::nav_msgs::OccupancyGrid>(
            kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
            true /* latched */);
    occupancy_grid_thread_ =
        std::thread(&Node::SpinOccupancyGridThreadForever, this);
  }

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  finish_trajectory_server_ = node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(options_.pose_publish_period_sec),
      &Node::PublishPoseAndScanMatchedPointCloud, this));
}

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  return map_builder_bridge_->HandleSubmapQuery(request, response);
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  return map_builder_bridge_->HandleFinishTrajectory(request, response);
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_->GetSubmapList());
}

void Node::PublishPoseAndScanMatchedPointCloud(
    const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  const carto::mapping::TrajectoryBuilder* trajectory_builder =
      map_builder_bridge_->map_builder()->GetTrajectoryBuilder(
          map_builder_bridge_->trajectory_id());
  const carto::mapping::TrajectoryBuilder::PoseEstimate last_pose_estimate =
      trajectory_builder->pose_estimate();
  if (carto::common::ToUniversal(last_pose_estimate.time) < 0) {
    return;
  }

  const Rigid3d tracking_to_local = last_pose_estimate.pose;
  const Rigid3d local_to_map =
      map_builder_bridge_->map_builder()
          ->sparse_pose_graph()
          ->GetLocalToGlobalTransform(*trajectory_builder->submaps());
  const Rigid3d tracking_to_map = local_to_map * tracking_to_local;

  geometry_msgs::TransformStamped stamped_transform;
  stamped_transform.header.stamp = ToRos(last_pose_estimate.time);

  // We only publish a point cloud if it has changed. It is not needed at high
  // frequency, and republishing it would be computationally wasteful.
  if (last_pose_estimate.time != last_scan_matched_point_cloud_time_) {
    scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
        carto::common::ToUniversal(last_pose_estimate.time),
        options_.tracking_frame,
        carto::sensor::TransformPointCloud(
            last_pose_estimate.point_cloud,
            tracking_to_local.inverse().cast<float>())));
    last_scan_matched_point_cloud_time_ = last_pose_estimate.time;
  } else {
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance.
    stamped_transform.header.stamp = ros::Time::now();
  }

  const auto published_to_tracking =
      map_builder_bridge_->tf_bridge()->LookupToTracking(
          last_pose_estimate.time, options_.published_frame);
  if (published_to_tracking != nullptr) {
    if (options_.provide_odom_frame) {
      std::vector<geometry_msgs::TransformStamped> stamped_transforms;

      stamped_transform.header.frame_id = options_.map_frame;
      stamped_transform.child_frame_id = options_.odom_frame;
      stamped_transform.transform = ToGeometryMsgTransform(local_to_map);
      stamped_transforms.push_back(stamped_transform);

      stamped_transform.header.frame_id = options_.odom_frame;
      stamped_transform.child_frame_id = options_.published_frame;
      stamped_transform.transform =
          ToGeometryMsgTransform(tracking_to_local * (*published_to_tracking));
      stamped_transforms.push_back(stamped_transform);

      tf_broadcaster_.sendTransform(stamped_transforms);
    } else {
      stamped_transform.header.frame_id = options_.map_frame;
      stamped_transform.child_frame_id = options_.published_frame;
      stamped_transform.transform =
          ToGeometryMsgTransform(tracking_to_map * (*published_to_tracking));
      tf_broadcaster_.sendTransform(stamped_transform);
    }
  }
}

void Node::SpinOccupancyGridThreadForever() {
  for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      carto::common::MutexLocker lock(&mutex_);
      if (terminating_) {
        return;
      }
    }
    if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
      continue;
    }
    const auto occupancy_grid = map_builder_bridge_->BuildOccupancyGrid();
    if (occupancy_grid != nullptr) {
      occupancy_grid_publisher_.publish(*occupancy_grid);
    }
  }
}

void Node::SpinForever() { ::ros::spin(); }

}  // namespace cartographer_ros
