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

#include <chrono>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/common/rate_timer.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/submaps.pb.h"
#include "cartographer/mapping_2d/global_trajectory_builder.h"
#include "cartographer/mapping_2d/local_trajectory_builder.h"
#include "cartographer/mapping_2d/sparse_pose_graph.h"
#include "cartographer/mapping_3d/global_trajectory_builder.h"
#include "cartographer/mapping_3d/local_trajectory_builder.h"
#include "cartographer/mapping_3d/local_trajectory_builder_options.h"
#include "cartographer/mapping_3d/sparse_pose_graph.h"
#include "cartographer/sensor/collator.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/map_writer.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/occupancy_grid.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/xray.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectorySubmapList.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

namespace carto = ::cartographer;
namespace proto = carto::sensor::proto;

using carto::transform::Rigid3d;
using carto::kalman_filter::PoseCovariance;

// TODO(hrapp): Support multi trajectory mapping.
constexpr int64 kTrajectoryBuilderId = 0;
constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;
constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

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

// Node that listens to all the sensor data that we are interested in and wires
// it up to the SLAM.
class Node {
 public:
  Node(const NodeOptions& options);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void SpinForever();
  void Initialize();

 private:
  void HandleSensorData(const string& sensor_id,
                        std::unique_ptr<carto::sensor::Data> sensor_data);
  bool HandleSubmapQuery(
      ::cartographer_ros_msgs::SubmapQuery::Request& request,
      ::cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleFinishTrajectory(
      ::cartographer_ros_msgs::FinishTrajectory::Request& request,
      ::cartographer_ros_msgs::FinishTrajectory::Response& response);

  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void PublishPoseAndScanMatchedPointCloud(
      const ::ros::WallTimerEvent& timer_event);
  void SpinOccupancyGridThreadForever();

  const NodeOptions options_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  TfBridge tf_bridge_;

  carto::common::Mutex mutex_;
  std::deque<carto::mapping::TrajectoryNode::ConstantData> constant_data_
      GUARDED_BY(mutex_);
  carto::mapping::MapBuilder map_builder_ GUARDED_BY(mutex_);
  carto::sensor::Collator sensor_collator_ GUARDED_BY(mutex_);
  SensorBridge sensor_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Subscriber horizontal_laser_scan_subscriber_;
  std::vector<::ros::Subscriber> laser_subscribers_3d_;
  ::ros::Subscriber odometry_subscriber_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::ServiceServer submap_query_server_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;
  carto::common::Time last_scan_matched_point_cloud_time_ =
      carto::common::Time::min();
  ::ros::ServiceServer finish_trajectory_server_;

  ::ros::Publisher occupancy_grid_publisher_;
  std::thread occupancy_grid_thread_;
  bool terminating_ = false GUARDED_BY(mutex_);

  // Time at which we last logged the rates of incoming sensor data.
  std::chrono::steady_clock::time_point last_sensor_data_rates_logging_time_;
  std::map<string, carto::common::RateTimer<>> rate_timers_;

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;
};

Node::Node(const NodeOptions& options)
    : options_(options),
      tf_buffer_(::ros::Duration(1000)),
      tf_(tf_buffer_),
      tf_bridge_(options_.tracking_frame, options_.lookup_transform_timeout_sec,
                 &tf_buffer_),
      map_builder_(options.map_builder_options, &constant_data_),
      sensor_bridge_(options.sensor_bridge_options, &tf_bridge_,
                     kTrajectoryBuilderId, &sensor_collator_) {}

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
  // Set of all topics we subscribe to. We use the non-remapped default names
  // which are unique.
  std::unordered_set<string> expected_sensor_identifiers;

  // For 2D SLAM, subscribe to exactly one horizontal laser.
  if (options_.use_horizontal_laser) {
    horizontal_laser_scan_subscriber_ = node_handle_.subscribe(
        kLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
            [this](const sensor_msgs::LaserScan::ConstPtr& msg) {
              sensor_bridge_.HandleLaserScanMessage(kLaserScanTopic, msg);
            }));
    expected_sensor_identifiers.insert(kLaserScanTopic);
  }
  if (options_.use_horizontal_multi_echo_laser) {
    horizontal_laser_scan_subscriber_ = node_handle_.subscribe(
        kMultiEchoLaserScanTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const sensor_msgs::MultiEchoLaserScan::ConstPtr&)>(
            [this](const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
              sensor_bridge_.HandleMultiEchoLaserScanMessage(
                  kMultiEchoLaserScanTopic, msg);
            }));
    expected_sensor_identifiers.insert(kMultiEchoLaserScanTopic);
  }

  // For 3D SLAM, subscribe to all 3D lasers.
  if (options_.num_lasers_3d > 0) {
    for (int i = 0; i < options_.num_lasers_3d; ++i) {
      string topic = kPointCloud2Topic;
      if (options_.num_lasers_3d > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      laser_subscribers_3d_.push_back(node_handle_.subscribe(
          topic, kInfiniteSubscriberQueueSize,
          boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
              [this, topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                sensor_bridge_.HandlePointCloud2Message(topic, msg);
              })));
      expected_sensor_identifiers.insert(topic);
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
              sensor_bridge_.HandleImuMessage(kImuTopic, msg);
            }));
    expected_sensor_identifiers.insert(kImuTopic);
  }

  if (options_.use_odometry_data) {
    odometry_subscriber_ = node_handle_.subscribe(
        kOdometryTopic, kInfiniteSubscriberQueueSize,
        boost::function<void(const nav_msgs::Odometry::ConstPtr&)>(
            [this](const nav_msgs::Odometry::ConstPtr& msg) {
              sensor_bridge_.HandleOdometryMessage(kOdometryTopic, msg);
            }));
    expected_sensor_identifiers.insert(kOdometryTopic);
  }

  // TODO(damonkohler): Add multi-trajectory support.
  CHECK_EQ(kTrajectoryBuilderId, map_builder_.AddTrajectoryBuilder());
  sensor_collator_.AddTrajectory(
      kTrajectoryBuilderId, expected_sensor_identifiers,
      [this](const string& sensor_id,
             std::unique_ptr<carto::sensor::Data> sensor_data) {
        HandleSensorData(sensor_id, std::move(sensor_data));
      });

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
  if (request.trajectory_id != 0) {
    return false;
  }

  carto::common::MutexLocker lock(&mutex_);
  // TODO(hrapp): return error messages and extract common code from MapBuilder.
  carto::mapping::Submaps* submaps =
      map_builder_.GetTrajectoryBuilder(kTrajectoryBuilderId)->submaps();
  if (request.submap_id < 0 || request.submap_id >= submaps->size()) {
    return false;
  }

  carto::mapping::proto::SubmapQuery::Response response_proto;
  response_proto.set_submap_id(request.submap_id);
  response_proto.set_submap_version(
      submaps->Get(request.submap_id)->end_laser_fan_index);
  const std::vector<carto::transform::Rigid3d> submap_transforms =
      map_builder_.sparse_pose_graph()->GetSubmapTransforms(*submaps);

  submaps->SubmapToProto(request.submap_id,
                         map_builder_.sparse_pose_graph()->GetTrajectoryNodes(),
                         submap_transforms[request.submap_id], &response_proto);

  response.submap_version = response_proto.submap_version();
  response.cells.insert(response.cells.begin(), response_proto.cells().begin(),
                        response_proto.cells().end());
  response.width = response_proto.width();
  response.height = response_proto.height();
  response.resolution = response_proto.resolution();

  response.slice_pose.position.x =
      response_proto.slice_pose().translation().x();
  response.slice_pose.position.y =
      response_proto.slice_pose().translation().y();
  response.slice_pose.position.z =
      response_proto.slice_pose().translation().z();
  response.slice_pose.orientation.x =
      response_proto.slice_pose().rotation().x();
  response.slice_pose.orientation.y =
      response_proto.slice_pose().rotation().y();
  response.slice_pose.orientation.z =
      response_proto.slice_pose().rotation().z();
  response.slice_pose.orientation.w =
      response_proto.slice_pose().rotation().w();
  return true;
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  // After shutdown, ROS messages will no longer be received and therefore not
  // pile up.
  //
  // TODO(whess): Continue with a new trajectory instead?
  ::ros::shutdown();
  carto::common::MutexLocker lock(&mutex_);
  // TODO(whess): Add multi-trajectory support.
  sensor_collator_.FinishTrajectory(kTrajectoryBuilderId);
  map_builder_.sparse_pose_graph()->RunFinalOptimization();

  const auto trajectory_nodes =
      map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
  if (trajectory_nodes.empty()) {
    LOG(WARNING) << "Map is empty and will not be saved.";
    return true;
  }

  if (options_.map_builder_options.use_trajectory_builder_2d()) {
    ::nav_msgs::OccupancyGrid occupancy_grid;
    BuildOccupancyGrid(trajectory_nodes, options_, &occupancy_grid);
    WriteOccupancyGridToPgmAndYaml(occupancy_grid, request.stem);
  }

  if (options_.map_builder_options.use_trajectory_builder_3d()) {
    WriteXRayImages(trajectory_nodes,
                    options_.map_builder_options.trajectory_builder_3d_options()
                        .submaps_options()
                        .high_resolution(),
                    request.stem);
  }
  return true;
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  const carto::mapping::Submaps* submaps =
      map_builder_.GetTrajectoryBuilder(kTrajectoryBuilderId)->submaps();
  const std::vector<carto::transform::Rigid3d> submap_transforms =
      map_builder_.sparse_pose_graph()->GetSubmapTransforms(*submaps);
  CHECK_EQ(submap_transforms.size(), submaps->size());

  ::cartographer_ros_msgs::TrajectorySubmapList ros_trajectory;
  for (int i = 0; i != submaps->size(); ++i) {
    ::cartographer_ros_msgs::SubmapEntry ros_submap;
    ros_submap.submap_version = submaps->Get(i)->end_laser_fan_index;
    ros_submap.pose = ToGeometryMsgPose(submap_transforms[i]);
    ros_trajectory.submap.push_back(ros_submap);
  }

  ::cartographer_ros_msgs::SubmapList ros_submap_list;
  ros_submap_list.header.stamp = ::ros::Time::now();
  ros_submap_list.header.frame_id = options_.map_frame;
  ros_submap_list.trajectory.push_back(ros_trajectory);
  submap_list_publisher_.publish(ros_submap_list);
}

void Node::PublishPoseAndScanMatchedPointCloud(
    const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  const carto::mapping::GlobalTrajectoryBuilderInterface::PoseEstimate
      last_pose_estimate =
          map_builder_.GetTrajectoryBuilder(kTrajectoryBuilderId)
              ->pose_estimate();
  if (carto::common::ToUniversal(last_pose_estimate.time) < 0) {
    return;
  }

  const Rigid3d tracking_to_local = last_pose_estimate.pose;
  const carto::mapping::Submaps* submaps =
      map_builder_.GetTrajectoryBuilder(kTrajectoryBuilderId)->submaps();
  const Rigid3d local_to_map =
      map_builder_.sparse_pose_graph()->GetLocalToGlobalTransform(*submaps);
  const Rigid3d tracking_to_map = local_to_map * tracking_to_local;

  geometry_msgs::TransformStamped stamped_transform;
  stamped_transform.header.stamp = ToRos(last_pose_estimate.time);

  const auto published_to_tracking = tf_bridge_.LookupToTracking(
      last_pose_estimate.time, options_.published_frame);
  if (published_to_tracking != nullptr) {
    if (options_.provide_odom_frame) {
      stamped_transform.header.frame_id = options_.map_frame;
      stamped_transform.child_frame_id = options_.odom_frame;
      stamped_transform.transform = ToGeometryMsgTransform(local_to_map);
      tf_broadcaster_.sendTransform(stamped_transform);

      stamped_transform.header.frame_id = options_.odom_frame;
      stamped_transform.child_frame_id = options_.published_frame;
      stamped_transform.transform =
          ToGeometryMsgTransform(tracking_to_local * (*published_to_tracking));
      tf_broadcaster_.sendTransform(stamped_transform);
    } else {
      stamped_transform.header.frame_id = options_.map_frame;
      stamped_transform.child_frame_id = options_.published_frame;
      stamped_transform.transform =
          ToGeometryMsgTransform(tracking_to_map * (*published_to_tracking));
      tf_broadcaster_.sendTransform(stamped_transform);
    }
  }

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
    const auto trajectory_nodes =
        map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
    if (trajectory_nodes.empty()) {
      continue;
    }
    ::nav_msgs::OccupancyGrid occupancy_grid;
    BuildOccupancyGrid(trajectory_nodes, options_, &occupancy_grid);
    occupancy_grid_publisher_.publish(occupancy_grid);
  }
}

void Node::HandleSensorData(const string& sensor_id,
                            std::unique_ptr<carto::sensor::Data> sensor_data) {
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    it =
        rate_timers_
            .emplace(std::piecewise_construct, std::forward_as_tuple(sensor_id),
                     std::forward_as_tuple(carto::common::FromSeconds(
                         kSensorDataRatesLoggingPeriodSeconds)))
            .first;
  }
  it->second.Pulse(sensor_data->time);

  if (std::chrono::steady_clock::now() - last_sensor_data_rates_logging_time_ >
      carto::common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_sensor_data_rates_logging_time_ = std::chrono::steady_clock::now();
  }

  auto* const trajectory_builder =
      map_builder_.GetTrajectoryBuilder(kTrajectoryBuilderId);
  switch (sensor_data->type) {
    case carto::sensor::Data::Type::kImu:
      trajectory_builder->AddImuData(sensor_data->time,
                                     sensor_data->imu.linear_acceleration,
                                     sensor_data->imu.angular_velocity);
      return;

    case carto::sensor::Data::Type::kLaserFan:
      if (options_.map_builder_options.use_trajectory_builder_2d()) {
        trajectory_builder->AddHorizontalLaserFan(sensor_data->time,
                                                  sensor_data->laser_fan);
      } else {
        CHECK(options_.map_builder_options.use_trajectory_builder_3d());
        trajectory_builder->AddLaserFan3D(sensor_data->time,
                                          sensor_data->laser_fan);
      }
      return;

    case carto::sensor::Data::Type::kOdometry:
      trajectory_builder->AddOdometerPose(sensor_data->time,
                                          sensor_data->odometry.pose,
                                          sensor_data->odometry.covariance);
      return;
  }
  LOG(FATAL);
}

void Node::SpinForever() { ::ros::spin(); }

void Run() {
  auto file_resolver =
      carto::common::make_unique<carto::common::ConfigurationFileResolver>(
          std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver), nullptr);

  Node node(CreateNodeOptions(&lua_parameter_dictionary));
  node.Initialize();
  node.SpinForever();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  ::cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Run();
  ::ros::shutdown();
}
