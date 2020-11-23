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
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "glog/logging.h"
#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include "visualization_msgs/msg/marker_array.hpp"

namespace cartographer_ros {

namespace {

cartographer_ros_msgs::msg::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::msg::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;
  topics.nav_sat_fix_topic = kNavSatFixTopic;
  topics.landmark_topic = kLandmarkTopic;
  return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Cartographer::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr),
    const int trajectory_id, const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, Cartographer* const node) {
  return node_handle->create_subscription<MessageType>(
      topic, rclcpp::SensorDataQoS(),
      [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
          (node->*handler)(trajectory_id, topic, msg);
      });
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Cartographer::Cartographer(
  const NodeOptions& node_options,
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder)
  : Node("cartographer_node"),
    node_options_(node_options)
{
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(node_handle_));
  tf_buffer_.reset(new tf2_ros::Buffer(get_clock(), ::tf2::durationFromSec(kTfBufferCacheTimeInSeconds)));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  map_builder_bridge_.reset(new cartographer_ros::MapBuilderBridge(node_options_, std::move(map_builder), tf_buffer_.get()));

  carto::common::MutexLocker lock(&mutex_);

  submap_list_publisher_ =
      this->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
          kSubmapListTopic, 10);
  trajectory_node_list_publisher_ =
      this->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kTrajectoryNodeListTopic, 10);
  landmark_poses_list_publisher_ =
      this->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kLandmarkPosesListTopic, 10);
  constraint_list_publisher_ =
      this->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kConstraintListTopic, 10);
  scan_matched_point_cloud_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        kScanMatchedPointCloudTopic, 10);

  auto submap_query_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Response> response) -> void
    {
      HandleSubmapQuery(request_header, request, response);
    };

  submap_query_server_ = create_service<cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName, submap_query_callback);

  auto start_trajectory_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory::Response> response) -> void
    {
      HandleStartTrajectory(request_header, request, response);
    };

  start_trajectory_server_ = create_service<cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName, start_trajectory_callback);

  auto finish_trajectory_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cartographer_ros_msgs::srv::FinishTrajectory::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::FinishTrajectory::Response> response) -> void
    {
      HandleFinishTrajectory(request_header, request, response);
    };

  finish_trajectory_server_ = create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName, finish_trajectory_callback);

  auto write_state_callback =
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Response> response) -> void
    {
      HandleWriteState(request_header, request, response);
    };

  write_state_server_ = create_service<cartographer_ros_msgs::srv::WriteState>(
      kWriteStateServiceName, write_state_callback);

  submap_list_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
    [this]() {
      PublishSubmapList();
    });

  trajectory_states_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
    [this]() {
      PublishTrajectoryStates();
    });

  trajectory_node_list_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishTrajectoryNodeList();
    });

  landmark_pose_list_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishLandmarkPosesList();
    });

  constrain_list_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
    [this]() {
      PublishConstraintList();
    });
}

Cartographer::~Cartographer() { FinishAllTrajectories(); }

::rclcpp::Node::SharedPtr Cartographer::node_handle() { return node_handle_; }

void Cartographer::HandleSubmapQuery(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Request> request,
    std::shared_ptr<::cartographer_ros_msgs::srv::SubmapQuery::Response> response) {

  (void)request_header;
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_->HandleSubmapQuery(request, response);
}

void Cartographer::PublishSubmapList() {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_->GetSubmapList(this->now()));
}

void Cartographer::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Cartographer::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Cartographer::PublishTrajectoryStates() {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      if (this->count_subscribers(kScanMatchedPointCloudTopic) > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_state.local_slam_data
                                ->range_data_in_local.returns.size());
        for (const Eigen::Vector3f point :
              trajectory_state.local_slam_data->range_data_in_local.returns) {
          Eigen::Vector4f point_time;
          point_time << point, 0.f;
          point_cloud.push_back(point_time);
        }
        scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_state.local_slam_data->time),
            node_options_.map_frame,
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_state.local_to_map.cast<float>())));
      }
      extrapolator.AddPose(trajectory_state.local_slam_data->time,
                           trajectory_state.local_slam_data->local_pose);
    }

    geometry_msgs::msg::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        FromRos(this->now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp = ToRos(now);

    const Rigid3d tracking_to_local = [&] {
      if (trajectory_state.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(extrapolator.ExtrapolatePose(now)));
      }
      return extrapolator.ExtrapolatePose(now);
    }();

    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    if (trajectory_state.published_to_tracking != nullptr) {
      if (trajectory_state.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_->sendTransform(stamped_transforms);
      } else {
        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_state.published_to_tracking));
        tf_broadcaster_->sendTransform(stamped_transform);
      }
    }
  }
}

void Cartographer::PublishTrajectoryNodeList() {
  if (this->count_subscribers(kTrajectoryNodeListTopic) > 0) {
    carto::common::MutexLocker lock(&mutex_);
    trajectory_node_list_publisher_->publish(
        map_builder_bridge_->GetTrajectoryNodeList(this->now()));
  }
}

void Cartographer::PublishLandmarkPosesList() {
  if (this->count_subscribers(kLandmarkPosesListTopic) > 0) {
    carto::common::MutexLocker lock(&mutex_);
    constraint_list_publisher_->publish(map_builder_bridge_->GetLandmarkPosesList(this->now()));
  }
}

void Cartographer::PublishConstraintList() {
  if (this->count_subscribers(kConstraintListTopic) > 0) {
    carto::common::MutexLocker lock(&mutex_);
    constraint_list_publisher_->publish(map_builder_bridge_->GetConstraintList(this->now()));
  }
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Cartographer::ComputeExpectedSensorIds(
    const TrajectoryOptions& options,
    const cartographer_ros_msgs::msg::SensorTopics& topics) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, topics.imu_topic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(
        SensorId{SensorType::ODOMETRY, topics.odometry_topic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, topics.nav_sat_fix_topic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

int Cartographer::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::msg::SensorTopics& topics) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options, topics);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, topics, trajectory_id);
  is_active_trajectory_[trajectory_id] = true;
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void Cartographer::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::msg::SensorTopics& topics,
                             const int trajectory_id) {
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
             &Cartographer::HandleLaserScanMessage, trajectory_id, topic, node_handle_,
             this),
         topic});
  }

  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
             &Cartographer::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             node_handle_, this),
         topic});
  }

  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
             &Cartographer::HandlePointCloud2Message, trajectory_id, topic,
             node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    std::string topic = topics.imu_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::Imu>(&Cartographer::HandleImuMessage,
                                                     trajectory_id, topic,
                                                     node_handle_, this),
         topic});
  }

  if (options.use_odometry) {
    std::string topic = topics.odometry_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Cartographer::HandleOdometryMessage,
                                                       trajectory_id, topic,
                                                       node_handle_, this),
         topic});
  }

  if (options.use_nav_sat) {
    std::string topic = topics.nav_sat_fix_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(&Cartographer::HandleNavSatFixMessage,
                                                       trajectory_id, topic,
                                                       node_handle_, this),
         topic});
  }

  if (options.use_landmarks) {
    std::string topic = topics.landmark_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(&Cartographer::HandleLandmarkMessage,
                                                       trajectory_id, topic,
                                                       node_handle_, this),
         topic});
  }
}

bool Cartographer::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Cartographer::ValidateTopicNames(
    const ::cartographer_ros_msgs::msg::SensorTopics& topics,
    const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options, topics)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::msg::StatusResponse Cartographer::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::msg::StatusResponse status_response;

  // First, check if we can actually finish the trajectory.
  if (map_builder_bridge_->GetFrozenTrajectoryIds().count(trajectory_id)) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is frozen.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    status_response.message = error;
    return status_response;
  }
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is not created yet.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    status_response.message = error;
    return status_response;
  }
  if (!is_active_trajectory_[trajectory_id]) {
    const std::string error = "Trajectory " + std::to_string(trajectory_id) +
                              " has already been finished.";
    LOG(ERROR) << error;
    status_response.code =
        cartographer_ros_msgs::msg::StatusCode::RESOURCE_EXHAUSTED;
    status_response.message = error;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  for (auto& entry : subscribers_[trajectory_id]) {
    subscribed_topics_.erase(entry.topic);
    LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
  }
  CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  CHECK(is_active_trajectory_.at(trajectory_id));
  map_builder_bridge_->FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  const std::string message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
  status_response.message = message;
  return status_response;
}

void Cartographer::HandleStartTrajectory(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<::cartographer_ros_msgs::srv::StartTrajectory::Request> request,
     std::shared_ptr<::cartographer_ros_msgs::srv::StartTrajectory::Response> response) {

  (void)request_header;
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request->options, &options) ||
      !ValidateTrajectoryOptions(options)) {
    const std::string error = "Invalid trajectory options.";
    LOG(ERROR) << error;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message = error;
  }
  else if (!ValidateTopicNames(request->topics, options)) {
    const std::string error = "Invalid topics.";
    LOG(ERROR) << error;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message = error;
  }
  else {
    response->trajectory_id = AddTrajectory(options, request->topics);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message = "Success.";
  }
}

void Cartographer::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options, DefaultSensorTopics());
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Cartographer::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id :
         ComputeExpectedSensorIds(bags_options.at(i), DefaultSensorTopics())) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Cartographer::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  is_active_trajectory_[trajectory_id] = true;
  return trajectory_id;
}

void Cartographer::HandleFinishTrajectory(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Request> request,
     std::shared_ptr<::cartographer_ros_msgs::srv::FinishTrajectory::Response> response) {

  (void)request_header;
  carto::common::MutexLocker lock(&mutex_);
  response->status = FinishTrajectoryUnderLock(request->trajectory_id);
}

void Cartographer::HandleWriteState(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<::cartographer_ros_msgs::srv::WriteState::Request> request,
     std::shared_ptr<::cartographer_ros_msgs::srv::WriteState::Response> response) {

  (void)request_header;
  carto::common::MutexLocker lock(&mutex_);
  if (map_builder_bridge_->SerializeState(request->filename)) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message = "State written to '" + request->filename + "'.";
  } else {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message = "Failed to write '" + request->filename + "'.";
  }
}

void Cartographer::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::msg::StatusCode::OK);
    }
  }
}

bool Cartographer::FinishTrajectory(const int trajectory_id) {
  carto::common::MutexLocker lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::msg::StatusCode::OK;
}

void Cartographer::RunFinalOptimization() {
  {
    carto::common::MutexLocker lock(&mutex_);
    for (const auto& entry : is_active_trajectory_) {
      CHECK(!entry.second);
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_->RunFinalOptimization();
}

void Cartographer::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Cartographer::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Cartographer::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

void Cartographer::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Cartographer::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Cartographer::HandleMultiEchoLaserScanMessage(
    int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Cartographer::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Cartographer::SerializeState(const std::string& filename) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(map_builder_bridge_->SerializeState(filename))
      << "Could not write state.";
}

void Cartographer::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_->LoadState(state_filename, load_frozen_state);
}


}  // namespace cartographer_ros
