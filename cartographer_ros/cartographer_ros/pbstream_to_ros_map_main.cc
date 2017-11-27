/*
 * Copyright 2017 The Cartographer Authors
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

#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(map_filestem, "map", "Stem of the output files.");
DEFINE_double(resolution, 0.05, "Resolution of a grid cell in the drawn map.");

namespace cartographer_ros {
namespace {

void FillSubmapSlice(
    const ::cartographer::transform::Rigid3d& global_submap_pose,
    const ::cartographer::mapping::proto::Submap& proto,
    ::cartographer::io::SubmapSlice* const submap_slice) {
  ::cartographer::mapping::proto::SubmapQuery::Response response;
  ::cartographer::transform::Rigid3d local_pose;
  if (proto.has_submap_3d()) {
    ::cartographer::mapping_3d::Submap submap(proto.submap_3d());
    local_pose = submap.local_pose();
    submap.ToResponseProto(global_submap_pose, &response);
  } else {
    ::cartographer::mapping_2d::Submap submap(proto.submap_2d());
    local_pose = submap.local_pose();
    submap.ToResponseProto(global_submap_pose, &response);
  }
  submap_slice->pose = global_submap_pose;

  auto& texture_proto = response.textures(0);
  const SubmapTexture::Pixels pixels = UnpackTextureData(
      texture_proto.cells(), texture_proto.width(), texture_proto.height());
  submap_slice->width = texture_proto.width();
  submap_slice->height = texture_proto.height();
  submap_slice->resolution = texture_proto.resolution();
  submap_slice->slice_pose =
      ::cartographer::transform::ToRigid3(texture_proto.slice_pose());
  submap_slice->surface =
      DrawTexture(pixels.intensity, pixels.alpha, texture_proto.width(),
                  texture_proto.height(), &submap_slice->cairo_data);
}

void Run(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);

  ::cartographer::mapping::proto::PoseGraph pose_graph;
  CHECK(reader.ReadProto(&pose_graph));

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  for (;;) {
    ::cartographer::mapping::proto::SerializedData proto;
    if (!reader.ReadProto(&proto)) {
      break;
    }
    if (proto.has_submap()) {
      const auto& submap = proto.submap();
      const ::cartographer::mapping::SubmapId id{
          submap.submap_id().trajectory_id(),
          submap.submap_id().submap_index()};
      const ::cartographer::transform::Rigid3d global_submap_pose =
          ::cartographer::transform::ToRigid3(
              pose_graph.trajectory(id.trajectory_id)
                  .submap(id.submap_index)
                  .pose());
      FillSubmapSlice(global_submap_pose, submap, &submap_slices[id]);
    }
  }
  CHECK(reader.eof());

  LOG(INFO) << "Generating combined map image from submap slices.";
  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

  ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

  ::cartographer::io::Image image(std::move(result.surface));
  WritePgm(image, resolution, &pgm_writer);

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
  WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_map_filestem.empty()) << "-map_filestem is missing.";

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_map_filestem,
                          FLAGS_resolution);
}
