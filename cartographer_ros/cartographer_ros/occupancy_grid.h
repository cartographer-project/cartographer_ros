#ifndef CARTOGRAPHER_ROS_OCCUPANCY_GRID_H_
#define CARTOGRAPHER_ROS_OCCUPANCY_GRID_H_

#include <fstream>

#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer_ros {

constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

struct OccupancyGridState {
  OccupancyGridState(::cartographer::io::UniqueCairoSurfacePtr surface,
                     Eigen::Array2f origin, Eigen::Array2i size)
      : surface(std::move(surface)), origin(origin), size(size) {}
  ::cartographer::io::UniqueCairoSurfacePtr surface;
  Eigen::Array2f origin;
  Eigen::Array2i size;
};

struct SubmapState {
  SubmapState()
      : surface(::cartographer::io::MakeUniqueCairoSurfacePtr(nullptr)) {}

  // Texture data.
  int width;
  int height;
  int version;
  double resolution;
  ::cartographer::transform::Rigid3d slice_pose;
  ::cartographer::io::UniqueCairoSurfacePtr surface;
  // Pixel data used by 'surface'. Must outlive 'surface'.
  std::vector<uint32_t> cairo_data;

  // Metadata.
  ::cartographer::transform::Rigid3d pose;
  int metadata_version = -1;
};

void CairoDrawEachSubmap(
    const double scale,
    std::map<::cartographer::mapping::SubmapId, SubmapState>* submaps,
    cairo_t* cr, std::function<void(const SubmapState&)> draw_callback);

OccupancyGridState DrawOccupancyGrid(
    std::map<::cartographer::mapping::SubmapId, SubmapState>* submaps,
    const double resolution);

void ExportOccupancyGrid(const OccupancyGridState& grid_state,
                         const double resolution, const std::string& stem);

void WriteOccupancyGridToPgm(const OccupancyGridState& grid_state,
                             const double resolution,
                             const std::string& filename);

void WriteOccupancyGridInfoToYaml(const OccupancyGridState& grid_state,
                                  const double resolution,
                                  const std::string& map_filename,
                                  const std::string& filename);
}

#endif  // CARTOGRAPHER_ROS_OCCUPANCY_GRID_H_
