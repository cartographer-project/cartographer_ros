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

#include <cstring>
#include <queue>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping/global_trajectory_builder_interface.h"
#include "cartographer/mapping/proto/submaps.pb.h"
#include "cartographer/mapping/sensor_collator.h"
#include "cartographer/mapping_2d/global_trajectory_builder.h"
#include "cartographer/mapping_2d/local_trajectory_builder.h"
#include "cartographer/mapping_2d/sparse_pose_graph.h"
#include "cartographer/mapping_3d/global_trajectory_builder.h"
#include "cartographer/mapping_3d/local_trajectory_builder.h"
#include "cartographer/mapping_3d/local_trajectory_builder_options.h"
#include "cartographer/mapping_3d/sparse_pose_graph.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectorySubmapList.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "gflags/gflags.h"
#include "glog/log_severity.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "msg_conversion.h"
#include "node_constants.h"
#include "time_conversion.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

using ::cartographer::transform::Rigid3d;
using ::cartographer::kalman_filter::PoseCovariance;
namespace proto = ::cartographer::sensor::proto;

// TODO(hrapp): Support multi trajectory mapping.
constexpr int64 kTrajectoryId = 0;
constexpr int kSubscriberQueueSize = 150;
constexpr int kSubmapPublishPeriodInUts = 300 * 10000ll;  // 300 milliseconds
constexpr int kPosePublishPeriodInUts = 5 * 10000ll;      // 5 milliseconds
constexpr double kMaxTransformDelaySeconds = 0.01;

// Unique default topic names. Expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "/scan";
constexpr char kMultiEchoLaserScanTopic[] = "/echoes";
constexpr char kPointCloud2Topic[] = "/points2";
constexpr char kImuTopic[] = "/imu";
constexpr char kOdometryTopic[] = "/odom";

Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return Rigid3d(Eigen::Vector3d(transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 transform.transform.translation.z),
                 Eigen::Quaterniond(transform.transform.rotation.w,
                                    transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z));
}

Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return Rigid3d(
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z),
      Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z));
}

PoseCovariance ToPoseCovariance(const boost::array<double, 36ul>& covariance) {
  return Eigen::Map<const Eigen::Matrix<double, 6, 6>>(covariance.data());
}

// TODO(hrapp): move to msg_conversion.cc.
geometry_msgs::Transform ToGeometryMsgTransform(const Rigid3d& rigid) {
  geometry_msgs::Transform transform;
  transform.translation.x = rigid.translation().x();
  transform.translation.y = rigid.translation().y();
  transform.translation.z = rigid.translation().z();
  transform.rotation.w = rigid.rotation().w();
  transform.rotation.x = rigid.rotation().x();
  transform.rotation.y = rigid.rotation().y();
  transform.rotation.z = rigid.rotation().z();
  return transform;
}

geometry_msgs::Pose ToGeometryMsgPose(const Rigid3d& rigid) {
  geometry_msgs::Pose pose;
  pose.position.x = rigid.translation().x();
  pose.position.y = rigid.translation().y();
  pose.position.z = rigid.translation().z();
  pose.orientation.w = rigid.rotation().w();
  pose.orientation.x = rigid.rotation().x();
  pose.orientation.y = rigid.rotation().y();
  pose.orientation.z = rigid.rotation().z();
  return pose;
}

// This type is a logical union, i.e. only one proto is actually filled in. It
// is only used for time ordering sensor data before passing it on.
enum class SensorType { kImu, kLaserScan, kLaserFan3D, kOdometry };
struct SensorData {
  SensorData(const string& init_frame_id, proto::Imu init_imu)
      : type(SensorType::kImu), frame_id(init_frame_id), imu(init_imu) {}
  SensorData(const string& init_frame_id, proto::LaserScan init_laser_scan)
      : type(SensorType::kLaserScan),
        frame_id(init_frame_id),
        laser_scan(init_laser_scan) {}
  SensorData(const string& init_frame_id, proto::LaserFan3D init_laser_fan_3d)
      : type(SensorType::kLaserFan3D),
        frame_id(init_frame_id),
        laser_fan_3d(init_laser_fan_3d) {}
  SensorData(const string& init_frame_id, const Rigid3d& init_pose,
             const PoseCovariance& init_covariance)
      : type(SensorType::kOdometry),
        frame_id(init_frame_id),
        odometry{init_pose, init_covariance} {}

  SensorType type;
  string frame_id;
  proto::Imu imu;
  proto::LaserScan laser_scan;
  proto::LaserFan3D laser_fan_3d;
  struct {
    Rigid3d pose;
    PoseCovariance covariance;
  } odometry;
};

// Node that listens to all the sensor data that we are interested in and wires
// it up to the SLAM.
class Node {
 public:
  Node();
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void SpinForever();
  void Initialize();

 private:
  void HandleSensorData(int64 timestamp,
                        std::unique_ptr<SensorData> sensor_data);
  void OdometryMessageCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void ImuMessageCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void LaserScanMessageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void MultiEchoLaserScanMessageCallback(
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void PointCloud2MessageCallback(
      const string& topic, const sensor_msgs::PointCloud2::ConstPtr& msg);

  void AddOdometry(int64 timestamp, const string& frame_id, const Rigid3d& pose,
                   const PoseCovariance& covariance);
  void AddImu(int64 timestamp, const string& frame_id, const proto::Imu& imu);
  void AddHorizontalLaserFan(int64 timestamp, const string& frame_id,
                             const proto::LaserScan& laser_scan);
  void AddLaserFan3D(int64 timestamp, const string& frame_id,
                     const proto::LaserFan3D& laser_fan_3d);

  // Returns a transform for 'frame_id' to 'tracking_frame_' if it exists at
  // 'time' or throws tf2::TransformException if it does not exist.
  Rigid3d LookupToTrackingTransformOrThrow(::cartographer::common::Time time,
                                           const string& frame_id);

  bool HandleSubmapQuery(
      ::cartographer_ros_msgs::SubmapQuery::Request& request,
      ::cartographer_ros_msgs::SubmapQuery::Response& response);

  void PublishSubmapList(int64 timestamp);
  void PublishPose(int64 timestamp);

  // TODO(hrapp): Pull out the common functionality between this and MapWriter
  // into an open sourcable MapWriter.
  ::cartographer::mapping::SensorCollator<SensorData> sensor_collator_;
  ros::NodeHandle node_handle_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber laser_subscriber_2d_;
  std::vector<ros::Subscriber> laser_subscribers_3d_;
  ros::Subscriber odometry_subscriber_;
  string tracking_frame_;
  string odom_frame_;
  string map_frame_;
  bool provide_odom_frame_;
  bool expect_odometry_data_;
  double laser_min_range_;
  double laser_max_range_;
  double laser_missing_echo_ray_length_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ::cartographer::common::ThreadPool thread_pool_;
  int64 last_pose_publish_timestamp_;
  ::cartographer::common::Mutex mutex_;
  std::unique_ptr<::cartographer::mapping::GlobalTrajectoryBuilderInterface>
      trajectory_builder_ GUARDED_BY(mutex_);
  std::deque<::cartographer::mapping::TrajectoryNode::ConstantData>
      constant_node_data_ GUARDED_BY(mutex_);
  std::unique_ptr<::cartographer::mapping::SparsePoseGraph> sparse_pose_graph_;

  ::ros::Publisher submap_list_publisher_;
  int64 last_submap_list_publish_timestamp_;
  ::ros::ServiceServer submap_query_server_;
};

Node::Node()
    : node_handle_("~"),
      tf_buffer_(ros::Duration(1000)),
      tf_(tf_buffer_),
      thread_pool_(10),
      last_submap_list_publish_timestamp_(0),
      last_pose_publish_timestamp_(0) {}

Rigid3d Node::LookupToTrackingTransformOrThrow(
    const ::cartographer::common::Time time, const string& frame_id) {
  return ToRigid3d(
      tf_buffer_.lookupTransform(tracking_frame_, frame_id, ToRos(time),
                                 ros::Duration(kMaxTransformDelaySeconds)));
}

void Node::OdometryMessageCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  auto sensor_data = ::cartographer::common::make_unique<SensorData>(
      msg->header.frame_id, ToRigid3d(msg->pose.pose),
      ToPoseCovariance(msg->pose.covariance));
  sensor_collator_.AddSensorData(
      kTrajectoryId,
      ::cartographer::common::ToUniversal(FromRos(msg->header.stamp)),
      kOdometryTopic, std::move(sensor_data));
}

void Node::AddOdometry(int64 timestamp, const string& frame_id,
                       const Rigid3d& pose, const PoseCovariance& covariance) {
  const ::cartographer::common::Time time =
      ::cartographer::common::FromUniversal(timestamp);
  trajectory_builder_->AddOdometerPose(time, pose, covariance);
}

void Node::ImuMessageCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  auto sensor_data = ::cartographer::common::make_unique<SensorData>(
      msg->header.frame_id, ToCartographer(*msg));
  sensor_collator_.AddSensorData(
      kTrajectoryId,
      ::cartographer::common::ToUniversal(FromRos(msg->header.stamp)),
      kImuTopic, std::move(sensor_data));
}

void Node::AddImu(const int64 timestamp, const string& frame_id,
                  const proto::Imu& imu) {
  const ::cartographer::common::Time time =
      ::cartographer::common::FromUniversal(timestamp);
  try {
    const Rigid3d sensor_to_tracking =
        LookupToTrackingTransformOrThrow(time, frame_id);
    CHECK(sensor_to_tracking.translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame. "
           "Transforming linear accelaration into the tracking frame will "
           "otherwise be imprecise.";
    trajectory_builder_->AddImuData(
        time, sensor_to_tracking.rotation() *
                  ::cartographer::transform::ToEigen(imu.linear_acceleration()),
        sensor_to_tracking.rotation() *
            ::cartographer::transform::ToEigen(imu.angular_velocity()));
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << "Cannot transform " << frame_id << " -> " << tracking_frame_
                 << ": " << ex.what();
  }
}

void Node::LaserScanMessageCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  auto sensor_data = ::cartographer::common::make_unique<SensorData>(
      msg->header.frame_id, ToCartographer(*msg));
  sensor_collator_.AddSensorData(
      kTrajectoryId,
      ::cartographer::common::ToUniversal(FromRos(msg->header.stamp)),
      kLaserScanTopic, std::move(sensor_data));
}

void Node::AddHorizontalLaserFan(const int64 timestamp, const string& frame_id,
                                 const proto::LaserScan& laser_scan) {
  const ::cartographer::common::Time time =
      ::cartographer::common::FromUniversal(timestamp);
  try {
    const Rigid3d sensor_to_tracking =
        LookupToTrackingTransformOrThrow(time, frame_id);
    const ::cartographer::sensor::LaserFan laser_fan =
        ::cartographer::sensor::ToLaserFan(laser_scan, laser_min_range_,
                                           laser_max_range_,
                                           laser_missing_echo_ray_length_);

    const auto laser_fan_3d = ::cartographer::sensor::TransformLaserFan3D(
        ::cartographer::sensor::ToLaserFan3D(laser_fan),
        sensor_to_tracking.cast<float>());
    trajectory_builder_->AddHorizontalLaserFan(time, laser_fan_3d);
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << "Cannot transform " << frame_id << " -> " << tracking_frame_
                 << ": " << ex.what();
  }
}

void Node::MultiEchoLaserScanMessageCallback(
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  auto sensor_data = ::cartographer::common::make_unique<SensorData>(
      msg->header.frame_id, ToCartographer(*msg));
  sensor_collator_.AddSensorData(
      kTrajectoryId,
      ::cartographer::common::ToUniversal(FromRos(msg->header.stamp)),
      kMultiEchoLaserScanTopic, std::move(sensor_data));
}

void Node::PointCloud2MessageCallback(
    const string& topic, const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_points;
  pcl::fromROSMsg(*msg, pcl_points);

  auto sensor_data = ::cartographer::common::make_unique<SensorData>(
      msg->header.frame_id, ToCartographer(pcl_points));
  sensor_collator_.AddSensorData(
      kTrajectoryId,
      ::cartographer::common::ToUniversal(FromRos(msg->header.stamp)), topic,
      std::move(sensor_data));
}

void Node::AddLaserFan3D(const int64 timestamp, const string& frame_id,
                         const proto::LaserFan3D& laser_fan_3d) {
  const ::cartographer::common::Time time =
      ::cartographer::common::FromUniversal(timestamp);
  try {
    const Rigid3d sensor_to_tracking =
        LookupToTrackingTransformOrThrow(time, frame_id);
    trajectory_builder_->AddLaserFan3D(
        time, ::cartographer::sensor::TransformLaserFan3D(
                  ::cartographer::sensor::FromProto(laser_fan_3d),
                  sensor_to_tracking.cast<float>()));
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << "Cannot transform " << frame_id << " -> " << tracking_frame_
                 << ": " << ex.what();
  }
}

void Node::Initialize() {
  auto file_resolver = ::cartographer::common::make_unique<
      ::cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  ::cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver), nullptr);

  tracking_frame_ = lua_parameter_dictionary.GetString("tracking_frame");
  odom_frame_ = lua_parameter_dictionary.GetString("odom_frame");
  map_frame_ = lua_parameter_dictionary.GetString("map_frame");
  provide_odom_frame_ = lua_parameter_dictionary.GetBool("provide_odom_frame");
  expect_odometry_data_ = lua_parameter_dictionary.GetBool("expect_odometry_data");
  laser_min_range_ = lua_parameter_dictionary.GetDouble("laser_min_range");
  laser_max_range_ = lua_parameter_dictionary.GetDouble("laser_max_range");
  laser_missing_echo_ray_length_ =
      lua_parameter_dictionary.GetDouble("laser_missing_echo_ray_length");

  // Set of all topics we subscribe to. We use the non-remapped default names
  // which are unique.
  std::unordered_set<string> expected_sensor_identifiers;

  // Subscribe to exactly one laser.
  const bool has_laser_scan_2d =
      lua_parameter_dictionary.HasKey("use_laser_scan_2d") &&
      lua_parameter_dictionary.GetBool("use_laser_scan_2d");
  const bool has_multi_echo_laser_scan_2d =
      lua_parameter_dictionary.HasKey("use_multi_echo_laser_scan_2d") &&
      lua_parameter_dictionary.GetBool("use_multi_echo_laser_scan_2d");
  const int num_lasers_3d =
      lua_parameter_dictionary.HasKey("num_lasers_3d")
          ? lua_parameter_dictionary.GetNonNegativeInt("num_lasers_3d")
          : 0;

  CHECK(has_laser_scan_2d + has_multi_echo_laser_scan_2d +
            (num_lasers_3d > 0) ==
        1)
      << "Parameters 'use_laser_scan_2d', 'use_multi_echo_laser_scan_2d' and "
         "'num_lasers_3d' are mutually exclusive, but one is required.";

  if (has_laser_scan_2d) {
    laser_subscriber_2d_ =
        node_handle_.subscribe(kLaserScanTopic, kSubscriberQueueSize,
                               &Node::LaserScanMessageCallback, this);
    expected_sensor_identifiers.insert(kLaserScanTopic);
  }
  if (has_multi_echo_laser_scan_2d) {
    laser_subscriber_2d_ =
        node_handle_.subscribe(kMultiEchoLaserScanTopic, kSubscriberQueueSize,
                               &Node::MultiEchoLaserScanMessageCallback, this);
    expected_sensor_identifiers.insert(kMultiEchoLaserScanTopic);
  }

  bool expect_imu_data = true;
  if (has_laser_scan_2d || has_multi_echo_laser_scan_2d) {
    auto sparse_pose_graph_2d = ::cartographer::common::make_unique<
        ::cartographer::mapping_2d::SparsePoseGraph>(
        ::cartographer::mapping::CreateSparsePoseGraphOptions(
            lua_parameter_dictionary.GetDictionary("sparse_pose_graph").get()),
        &thread_pool_, &constant_node_data_);
    auto options =
        ::cartographer::mapping_2d::CreateLocalTrajectoryBuilderOptions(
            lua_parameter_dictionary.GetDictionary("trajectory_builder").get());
    expect_imu_data = options.expect_imu_data();
    trajectory_builder_ = ::cartographer::common::make_unique<
        ::cartographer::mapping_2d::GlobalTrajectoryBuilder>(
        options, sparse_pose_graph_2d.get());
    sparse_pose_graph_ = std::move(sparse_pose_graph_2d);
  }

  if (num_lasers_3d > 0) {
    for (int i = 0; i < num_lasers_3d; ++i) {
      string topic = kPointCloud2Topic;
      if (num_lasers_3d > 1) {
        topic += "_" + std::to_string(i + 1);
      }
      laser_subscribers_3d_.push_back(node_handle_.subscribe(
          topic, kSubscriberQueueSize,
          boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>(
              [this, topic](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                PointCloud2MessageCallback(topic, msg);
              })));
      expected_sensor_identifiers.insert(topic);
    }
    auto sparse_pose_graph_3d = ::cartographer::common::make_unique<
        ::cartographer::mapping_3d::SparsePoseGraph>(
        ::cartographer::mapping::CreateSparsePoseGraphOptions(
            lua_parameter_dictionary.GetDictionary("sparse_pose_graph").get()),
        &thread_pool_, &constant_node_data_);
    trajectory_builder_ = ::cartographer::common::make_unique<
        ::cartographer::mapping_3d::GlobalTrajectoryBuilder>(
        ::cartographer::mapping_3d::CreateLocalTrajectoryBuilderOptions(
            lua_parameter_dictionary.GetDictionary("trajectory_builder").get()),
        sparse_pose_graph_3d.get());
    sparse_pose_graph_ = std::move(sparse_pose_graph_3d);
  }
  CHECK(sparse_pose_graph_ != nullptr);

  // Maybe subscribe to the IMU.
  if (expect_imu_data) {
    imu_subscriber_ = node_handle_.subscribe(kImuTopic, kSubscriberQueueSize,
                                             &Node::ImuMessageCallback, this);
    expected_sensor_identifiers.insert(kImuTopic);
  }

  if (expect_odometry_data_) {
    odometry_subscriber_ = node_handle_.subscribe(
      kOdometryTopic, kSubscriberQueueSize, &Node::OdometryMessageCallback, this);
    expected_sensor_identifiers.insert(kOdometryTopic);
  }

  sensor_collator_.AddTrajectory(
      kTrajectoryId, expected_sensor_identifiers,
      [this](const int64 timestamp, std::unique_ptr<SensorData> sensor_data) {
        HandleSensorData(timestamp, std::move(sensor_data));
      });

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, 10);
  submap_query_server_ = node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this);
}

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  if (request.trajectory_id != 0) {
    return false;
  }

  ::cartographer::common::MutexLocker lock(&mutex_);
  // TODO(hrapp): return error messages and extract common code from MapBuilder.
  ::cartographer::mapping::Submaps* submaps = trajectory_builder_->submaps();
  if (request.submap_id < 0 || request.submap_id >= submaps->size()) {
    return false;
  }

  ::cartographer::mapping::proto::SubmapQuery::Response response_proto;
  response_proto.set_submap_id(request.submap_id);
  response_proto.set_submap_version(
      submaps->Get(request.submap_id)->end_laser_fan_index);
  const std::vector<::cartographer::transform::Rigid3d> submap_transforms =
      sparse_pose_graph_->GetSubmapTransforms(*submaps);

  submaps->SubmapToProto(request.submap_id,
                         sparse_pose_graph_->GetTrajectoryNodes(),
                         submap_transforms[request.submap_id], &response_proto);

  response.submap_version = response_proto.submap_version();
  response.cells.insert(response.cells.begin(), response_proto.cells().begin(),
                        response_proto.cells().end());
  response.width = response_proto.width();
  response.height = response_proto.height();
  response.resolution = response_proto.resolution();

  auto pose = ::cartographer::transform::ToRigid3(response_proto.slice_pose());
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

void Node::PublishSubmapList(const int64 timestamp) {
  ::cartographer::common::MutexLocker lock(&mutex_);
  const ::cartographer::mapping::Submaps* submaps =
      trajectory_builder_->submaps();
  const std::vector<::cartographer::transform::Rigid3d> submap_transforms =
      sparse_pose_graph_->GetSubmapTransforms(*submaps);
  CHECK_EQ(submap_transforms.size(), submaps->size());

  ::cartographer_ros_msgs::TrajectorySubmapList ros_trajectory;
  for (int i = 0; i != submaps->size(); ++i) {
    ::cartographer_ros_msgs::SubmapEntry ros_submap;
    ros_submap.submap_version = submaps->Get(i)->end_laser_fan_index;
    ros_submap.pose = ToGeometryMsgPose(submap_transforms[i]);
    ros_trajectory.submap.push_back(ros_submap);
  }

  ::cartographer_ros_msgs::SubmapList ros_submap_list;
  ros_submap_list.header.stamp =
      ToRos(::cartographer::common::FromUniversal(timestamp));
  ros_submap_list.header.frame_id = map_frame_;
  ros_submap_list.trajectory.push_back(ros_trajectory);
  submap_list_publisher_.publish(ros_submap_list);
  last_submap_list_publish_timestamp_ = timestamp;
}

void Node::PublishPose(const int64 timestamp) {
  const ::cartographer::common::Time time =
      ::cartographer::common::FromUniversal(timestamp);
  const Rigid3d tracking_to_local = trajectory_builder_->pose_estimate().pose;
  const ::cartographer::mapping::Submaps* submaps =
      trajectory_builder_->submaps();
  const Rigid3d local_to_map =
      sparse_pose_graph_->GetLocalToGlobalTransform(*submaps);
  const Rigid3d tracking_to_map = local_to_map * tracking_to_local;

  geometry_msgs::TransformStamped stamped_transform;
  stamped_transform.header.stamp = ToRos(time);
  stamped_transform.header.frame_id = map_frame_;
  stamped_transform.child_frame_id = odom_frame_;

  if (provide_odom_frame_) {
    ::cartographer::common::MutexLocker lock(&mutex_);
    stamped_transform.transform = ToGeometryMsgTransform(local_to_map);
    tf_broadcaster_.sendTransform(stamped_transform);

    stamped_transform.header.frame_id = odom_frame_;
    stamped_transform.child_frame_id = tracking_frame_;
    stamped_transform.transform = ToGeometryMsgTransform(tracking_to_local);
    tf_broadcaster_.sendTransform(stamped_transform);
  } else {
    try {
      const Rigid3d tracking_to_odom =
          LookupToTrackingTransformOrThrow(time, odom_frame_).inverse();
      const Rigid3d odom_to_map = tracking_to_map * tracking_to_odom.inverse();
      stamped_transform.transform = ToGeometryMsgTransform(odom_to_map);
      tf_broadcaster_.sendTransform(stamped_transform);
    } catch (const tf2::TransformException& ex) {
      LOG(WARNING) << "Cannot transform " << tracking_frame_ << " -> "
                   << odom_frame_ << ": " << ex.what();
    }
  }
  last_pose_publish_timestamp_ = timestamp;
}

void Node::HandleSensorData(const int64 timestamp,
                            std::unique_ptr<SensorData> sensor_data) {
  if (last_submap_list_publish_timestamp_ + kSubmapPublishPeriodInUts <
      timestamp) {
    PublishSubmapList(timestamp);
  }

  if (last_pose_publish_timestamp_ + kPosePublishPeriodInUts < timestamp) {
    PublishPose(timestamp);
  }

  switch (sensor_data->type) {
    case SensorType::kImu:
      AddImu(timestamp, sensor_data->frame_id, sensor_data->imu);
      return;

    case SensorType::kLaserScan:
      AddHorizontalLaserFan(timestamp, sensor_data->frame_id,
                            sensor_data->laser_scan);
      return;

    case SensorType::kLaserFan3D:
      AddLaserFan3D(timestamp, sensor_data->frame_id,
                    sensor_data->laser_fan_3d);
      return;

    case SensorType::kOdometry:
      AddOdometry(timestamp, sensor_data->frame_id, sensor_data->odometry.pose,
                  sensor_data->odometry.covariance);
      return;
  }
  LOG(FATAL);
}

void Node::SpinForever() { ros::spin(); }

void Run() {
  Node node;
  node.Initialize();
  node.SpinForever();
}

const char* GetBasename(const char* filepath) {
  const char* base = strrchr(filepath, '/');
  return base ? (base + 1) : filepath;
}

// Makes Google logging use ROS logging for output while an instance of this
// class exists.
class ScopedRosLogSink : public google::LogSink {
 public:
  ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
  ~ScopedRosLogSink() override { RemoveLogSink(this); }

  void send(google::LogSeverity severity, const char* filename,
            const char* base_filename, int line, const struct ::tm* tm_time,
            const char* message, size_t message_len) override {
    const std::string message_string = google::LogSink::ToString(
        severity, GetBasename(filename), line, tm_time, message, message_len);
    switch (severity) {
      case google::GLOG_INFO:
        ROS_INFO_STREAM(message_string);
        break;

      case google::GLOG_WARNING:
        ROS_WARN_STREAM(message_string);
        break;

      case google::GLOG_ERROR:
        ROS_ERROR_STREAM(message_string);
        break;

      case google::GLOG_FATAL:
        ROS_FATAL_STREAM(message_string);
        will_die_ = true;
        break;
    }
  }

  void WaitTillSent() override {
    if (will_die_) {
      // Give ROS some time to actually publish our message.
      std::this_thread::sleep_for(
          ::cartographer::common::FromMilliseconds(1000));
    }
  }

 private:
  bool will_die_;
};

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ros::init(argc, argv, "cartographer_node");
  ros::start();

  ::cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Run();
  ros::shutdown();
  return 0;
}
