/*
 * Copyright 2018 The Cartographer Authors
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

// Publishes a frozen nav_msgs/OccupancyGrid map from serialized submaps.

#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(map_topic, "map", "Name of the published map topic.");
DEFINE_string(map_frame_id, "map", "Frame ID of the published map.");
DEFINE_double(resolution, 0.05, "Resolution of a grid cell in the drawn map.");

namespace cartographer_ros {
namespace {

void Run(const std::string& pbstream_filename, const std::string& map_topic,
         const std::string& map_frame_id, const double resolution) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::proto::SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    if (proto.has_submap()) {
      const auto& submap = proto.submap();
      const ::cartographer::mapping::SubmapId id{
          submap.submap_id().trajectory_id(),
          submap.submap_id().submap_index()};
      const ::cartographer::transform::Rigid3d global_submap_pose =
          ::cartographer::transform::ToRigid3(deserializer.pose_graph()
                                                  .trajectory(id.trajectory_id)
                                                  .submap(id.submap_index)
                                                  .pose());
      FillSubmapSlice(global_submap_pose, submap, &submap_slices[id]);
    }
  }
  CHECK(reader.eof());

  ::ros::NodeHandle node_handle("");
  ::ros::Publisher pub = node_handle.advertise<nav_msgs::OccupancyGrid>(
      map_topic, kLatestOnlyPublisherQueueSize, true /*latched */);

  LOG(INFO) << "Generating combined map image from submap slices.";
  const auto painted_slices =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution, FLAGS_map_frame_id, ros::Time::now());

  LOG(INFO) << "Publishing occupancy grid topic " << map_topic
            << " (frame_id: " << map_frame_id
            << ", resolution:" << std::to_string(resolution) << ").";
  pub.publish(*msg_ptr);
  ::ros::spin();
  ::ros::shutdown();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";

  ::ros::init(argc, argv, "cartographer_pbstream_map_publisher");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_map_topic,
                          FLAGS_map_frame_id, FLAGS_resolution);
}
