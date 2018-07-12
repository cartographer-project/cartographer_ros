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

#include "cartographer_ros/ros_map_writing_points_processor.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/image.h"
#include "cartographer/io/probability_grid_points_processor.h"
#include "cartographer_ros/ros_map.h"

namespace cartographer_ros {

RosMapWritingPointsProcessor::RosMapWritingPointsProcessor(
    const double resolution,
    const ::cartographer::mapping::proto::
        ProbabilityGridRangeDataInserterOptions2D& range_data_inserter_options,
    ::cartographer::io::FileWriterFactory file_writer_factory,
    const std::string& filestem,
    ::cartographer::io::PointsProcessor* const next)
    : filestem_(filestem),
      next_(next),
      file_writer_factory_(file_writer_factory),
      range_data_inserter_(range_data_inserter_options),
      probability_grid_(::cartographer::io::CreateProbabilityGrid(
          resolution, &conversion_tables_)) {}

std::unique_ptr<RosMapWritingPointsProcessor>
RosMapWritingPointsProcessor::FromDictionary(
    ::cartographer::io::FileWriterFactory file_writer_factory,
    ::cartographer::common::LuaParameterDictionary* const dictionary,
    ::cartographer::io::PointsProcessor* const next) {
  return ::cartographer::common::make_unique<RosMapWritingPointsProcessor>(
      dictionary->GetDouble("resolution"),
      ::cartographer::mapping::CreateProbabilityGridRangeDataInserterOptions2D(
          dictionary->GetDictionary("range_data_inserter").get()),
      file_writer_factory, dictionary->GetString("filestem"), next);
}

void RosMapWritingPointsProcessor::Process(
    std::unique_ptr<::cartographer::io::PointsBatch> batch) {
  range_data_inserter_.Insert({batch->origin, batch->points, {}},
                              &probability_grid_);
  next_->Process(std::move(batch));
}

::cartographer::io::PointsProcessor::FlushResult
RosMapWritingPointsProcessor::Flush() {
  Eigen::Array2i offset;
  std::unique_ptr<::cartographer::io::Image> image =
      ::cartographer::io::DrawProbabilityGrid(probability_grid_, &offset);
  if (image != nullptr) {
    auto pgm_writer = file_writer_factory_(filestem_ + ".pgm");
    const std::string pgm_filename = pgm_writer->GetFilename();
    const auto& limits = probability_grid_.limits();
    image->Rotate90DegreesClockwise();

    WritePgm(*image, limits.resolution(), pgm_writer.get());
    CHECK(pgm_writer->Close());

    const Eigen::Vector2d origin(
        limits.max().x() - (offset.y() + image->width()) * limits.resolution(),
        limits.max().y() -
            (offset.x() + image->height()) * limits.resolution());
    auto yaml_writer = file_writer_factory_(filestem_ + ".yaml");
    WriteYaml(limits.resolution(), origin, pgm_filename, yaml_writer.get());
    CHECK(yaml_writer->Close());
  }

  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "ROS map writing must be configured to occur after any "
                    "stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL);
  // The following unreachable return statement is needed to avoid a GCC bug
  // described at https://gcc.gnu.org/bugzilla/show_bug.cgi?id=81508
  return FlushResult::kFinished;
}

}  // namespace cartographer_ros
