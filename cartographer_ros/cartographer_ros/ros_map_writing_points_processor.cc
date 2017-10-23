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

namespace cartographer_ros {

namespace {

void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer) {
  // Flipping the image into ROS coordinate frame.
  const std::string header = "P5\n# Cartographer map; " +
                             std::to_string(resolution) + " m/pixel\n" +
                             std::to_string(image.height()) + " " +
                             std::to_string(image.width()) + "\n255\n";
  file_writer->Write(header.data(), header.size());
  for (int x = 0; x < image.width(); ++x) {
    for (int y = image.height() - 1; y >= 0; --y) {
      const char color = image.GetPixel(x, y)[0];
      file_writer->Write(&color, 1);
    }
  }
}

void WriteYaml(const ::cartographer::io::Image& image,
               const ::cartographer::mapping_2d::MapLimits& limits,
               const string& pgm_filename, const Eigen::Array2i& offset,
               ::cartographer::io::FileWriter* file_writer) {
  const double resolution = limits.resolution();
  const double x_offset =
      limits.max().x() - (offset.y() + image.height()) * resolution;
  const double y_offset =
      limits.max().y() - (offset.x() + image.width()) * resolution;
  // Magic constants taken directly from ros map_saver code:
  // https://github.com/ros-planning/navigation/blob/ac41d2480c4cf1602daf39a6e9629142731d92b0/map_server/src/map_saver.cpp#L114
  const std::string output =
      "image: " + pgm_filename + "\n" +
      "resolution: " + std::to_string(resolution) + "\n" + "origin: [" +
      std::to_string(x_offset) + ", " + std::to_string(y_offset) +
      ", 0.]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  file_writer->Write(output.data(), output.size());
}

}  // namespace

RosMapWritingPointsProcessor::RosMapWritingPointsProcessor(
    const double resolution,
    const ::cartographer::mapping_2d::proto::RangeDataInserterOptions&
        range_data_inserter_options,
    ::cartographer::io::FileWriterFactory file_writer_factory,
    const string& filestem, ::cartographer::io::PointsProcessor* const next)
    : filestem_(filestem),
      next_(next),
      file_writer_factory_(file_writer_factory),
      range_data_inserter_(range_data_inserter_options),
      probability_grid_(::cartographer::io::CreateProbabilityGrid(resolution)) {
}

std::unique_ptr<RosMapWritingPointsProcessor>
RosMapWritingPointsProcessor::FromDictionary(
    ::cartographer::io::FileWriterFactory file_writer_factory,
    ::cartographer::common::LuaParameterDictionary* const dictionary,
    ::cartographer::io::PointsProcessor* const next) {
  return ::cartographer::common::make_unique<RosMapWritingPointsProcessor>(
      dictionary->GetDouble("resolution"),
      ::cartographer::mapping_2d::CreateRangeDataInserterOptions(
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
    WritePgm(*image, probability_grid_.limits().resolution(), pgm_writer.get());
    CHECK(pgm_writer->Close());

    auto yaml_writer = file_writer_factory_(filestem_ + ".yaml");
    WriteYaml(*image, probability_grid_.limits(), pgm_filename, offset,
              yaml_writer.get());
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
}

}  // namespace cartographer_ros
