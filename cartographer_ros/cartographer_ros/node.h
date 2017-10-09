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

#ifndef CARTOGRAPHER_ROS_NODE_H_
#define CARTOGRAPHER_ROS_NODE_H_

#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/SensorTopics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const NodeOptions& node_options, tf2_ros::Buffer* tf_buffer);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();
  // Finishes a single given trajectory.
  void FinishTrajectory(int trajectory_id);

  // Runs final optimization. All trajectories have to be finished when calling.
  void RunFinalOptimization();

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // Compute the default topics for the given 'options'.
  std::unordered_set<string> ComputeDefaultTopics(
      const TrajectoryOptions& options);

  // Adds a trajectory for offline processing, i.e. not listening to topics.
  int AddOfflineTrajectory(
      const std::unordered_set<string>& expected_sensor_ids,
      const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleImuMessage(int trajectory_id, const string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(int trajectory_id, const string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  // Serializes the complete Node state.
  void SerializeState(const string& filename);

  // Loads a persisted state to use as a map.
  void LoadMap(const std::string& map_filename);

  ::ros::NodeHandle* node_handle();

 private:
  struct Subscriber {
    ::ros::Subscriber subscriber;

    // ::ros::Subscriber::getTopic() does not necessarily return the same string
    // it was given in its constructor. Since we rely on the topic name as the
    // unique identifier of a subscriber, we remember it ourselves.
    string topic;
  };

  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);
  bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);
  // Returns the set of topic names we want to subscribe to.
  std::unordered_set<string> ComputeExpectedTopics(
      const TrajectoryOptions& options,
      const cartographer_ros_msgs::SensorTopics& topics);
  int AddTrajectory(const TrajectoryOptions& options,
                    const cartographer_ros_msgs::SensorTopics& topics);
  void LaunchSubscribers(const TrajectoryOptions& options,
                         const cartographer_ros_msgs::SensorTopics& topics,
                         int trajectory_id);
  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
  void SpinOccupancyGridThreadForever();
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const ::cartographer_ros_msgs::SensorTopics& topics,
                          const TrajectoryOptions& options);

  const NodeOptions node_options_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  cartographer::common::Mutex mutex_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
  // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<::ros::ServiceServer> service_servers_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;

  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(double rangefinder_sampling_ratio,
                             double odometry_sampling_ratio,
                             double imu_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          imu_sampler(imu_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
  };

  // These are keyed with 'trajectory_id'.
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_NODE_H_
