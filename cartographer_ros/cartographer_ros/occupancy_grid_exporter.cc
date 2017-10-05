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
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/occupancy_grid.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(pbstream_filename, "",
              "Proto stream file containing the pose graph.");
DEFINE_double(resolution, 0.05, "Map Resolution");
DEFINE_string(stem, "map", "Prefix to export a map and meta data");

namespace cartographer_ros {

namespace {
namespace carto = ::cartographer;

void FillSubmapState(const carto::transform::Rigid3d& submap_pose,
                     const carto::mapping::proto::Submap& proto,
                     SubmapState* submap_state) {
  auto submap_2d = proto.submap_2d();
  auto probability_grid =
      carto::mapping_2d::ProbabilityGrid(submap_2d.probability_grid());
  Eigen::Array2i offset;
  carto::mapping_2d::CellLimits cell_limits;

  probability_grid.ComputeCroppedLimits(&offset, &cell_limits);

  const double resolution = probability_grid.limits().resolution();
  const double max_x =
      probability_grid.limits().max().x() - resolution * offset.y();
  const double max_y =
      probability_grid.limits().max().y() - resolution * offset.x();

  submap_state->pose = submap_pose;
  submap_state->metadata_version = submap_2d.num_range_data();
  submap_state->width = cell_limits.num_x_cells;
  submap_state->height = cell_limits.num_y_cells;
  submap_state->resolution = resolution;
  submap_state->slice_pose =
      carto::transform::ToRigid3(submap_2d.local_pose()).inverse() *
      carto::transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.));

  // Properly dealing with a non-common stride would make this code much more
  // complicated. Let's check that it is not needed.
  const int expected_stride = 4 * submap_state->width;
  CHECK_EQ(expected_stride,
           cairo_format_stride_for_width(kCairoFormat, submap_state->width));
  submap_state->cairo_data.clear();

  for (const Eigen::Array2i& xy_index :
       carto::mapping_2d::XYIndexRangeIterator(cell_limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      // We would like to add 'delta' but this is not possible using a value and
      // alpha. We use premultiplied alpha, so when 'delta' is positive we can
      // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
      // zero, and use 'alpha' to subtract. This is only correct when the pixel
      // is currently white, so walls will look too gray. This should be hard to
      // detect visually for the user, though.
      const int delta =
          128 - carto::mapping::ProbabilityToLogOddsInteger(
                    probability_grid.GetProbability(xy_index + offset));
      const uint8 alpha = delta > 0 ? 0 : -delta;
      const uint8 value = delta > 0 ? delta : 0;
      const uint8 observed = (value == 0 && alpha == 0) ? 0 : 255;
      submap_state->cairo_data.push_back((alpha << 24) | (value << 16) |
                                         (observed << 8) | 0);

    } else {
      constexpr uint8 kUnknownLogOdds = 0;
      const uint value = static_cast<uint8>(kUnknownLogOdds);
      const uint alpha = 0;
      const uint8 observed = 0;
      submap_state->cairo_data.push_back((alpha << 24) | (value << 16) |
                                         (observed << 8) | 0);
    }
  }

  submap_state->surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create_for_data(
          reinterpret_cast<unsigned char*>(submap_state->cairo_data.data()),
          kCairoFormat, submap_state->width, submap_state->height,
          expected_stride));
  CHECK_EQ(cairo_surface_status(submap_state->surface.get()),
           CAIRO_STATUS_SUCCESS)
      << cairo_status_to_string(
          cairo_surface_status(submap_state->surface.get()));
  return;
}

void Run(const string& pbstream_filename, const string& stem,
         const double resolution) {
  carto::io::ProtoStreamReader reader(pbstream_filename);

  // Skip Pose graph
  carto::mapping::proto::SparsePoseGraph pose_graph;
  CHECK(reader.ReadProto(&pose_graph));

  // Load submaps from pbstream
  LOG(INFO) << "Loading submaps from serialized data";
  std::map<carto::mapping::SubmapId, SubmapState> submaps;
  for (;;) {
    carto::mapping::proto::SerializedData proto;
    if (!reader.ReadProto(&proto)) {
      break;
    }
    if (proto.has_submap()) {
      auto submap = proto.submap();

      // Supoort only 2d for now
      if (!submap.has_submap_2d()) {
        return;
      }

      carto::mapping::SubmapId id{submap.submap_id().trajectory_id(),
                                  submap.submap_id().submap_index()};
      const carto::transform::Rigid3d submap_pose = carto::transform::ToRigid3(
          pose_graph.trajectory(submap.submap_id().trajectory_id())
              .submap(submap.submap_id().submap_index())
              .pose());
      SubmapState& submap_state = submaps[id];
      FillSubmapState(submap_pose, submap, &submap_state);
    }
  }
  CHECK(reader.eof());

  // Generate Occupancy Grid
  LOG(INFO) << "Generate Occupanct Grid from submaps";
  auto grid_state = DrawOccupancyGrid(&submaps, resolution);

  // Export map and meta data
  ExportOccupancyGrid(grid_state, resolution, stem);
}
}
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";

  LOG(INFO) << "Pbstream: " << FLAGS_pbstream_filename;
  LOG(INFO) << "Stem: " << FLAGS_stem;
  LOG(INFO) << "Resolution: " << FLAGS_resolution;

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_stem,
                          FLAGS_resolution);
}
