
#include "cartographer_ros/occupancy_grid.h"

Eigen::Affine3d ToEigen(const ::cartographer::transform::Rigid3d& rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}

namespace cartographer_ros {

void CairoDrawEachSubmap(
    const double scale,
    std::map<::cartographer::mapping::SubmapId, SubmapState>* submaps,
    cairo_t* cr, std::function<void(const SubmapState&)> draw_callback) {
  cairo_scale(cr, scale, scale);

  for (auto& pair : *submaps) {
    auto& submap_state = pair.second;
    if (submap_state.surface == nullptr) {
      return;
    }
    const Eigen::Matrix4d homo =
        ToEigen(submap_state.pose * submap_state.slice_pose).matrix();

    cairo_save(cr);
    cairo_matrix_t matrix;
    cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                      homo(0, 3), -homo(1, 3));
    cairo_transform(cr, &matrix);

    const double submap_resolution = submap_state.resolution;
    cairo_scale(cr, submap_resolution, submap_resolution);
    draw_callback(submap_state);
    cairo_restore(cr);
  }
}

OccupancyGridState DrawOccupancyGrid(
    std::map<::cartographer::mapping::SubmapId, SubmapState>* submaps,
    const double resolution) {
  Eigen::AlignedBox2f bounding_box;
  {
    auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
        cairo_image_surface_create(kCairoFormat, 1, 1));
    auto cr =
        ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    const auto update_bounding_box = [&bounding_box, &cr](double x, double y) {
      cairo_user_to_device(cr.get(), &x, &y);
      bounding_box.extend(Eigen::Vector2f(x, y));
    };

    CairoDrawEachSubmap(
        1. / resolution, submaps, cr.get(),
        [&update_bounding_box, &bounding_box](const SubmapState& submap_state) {
          update_bounding_box(0, 0);
          update_bounding_box(submap_state.width, 0);
          update_bounding_box(0, submap_state.height);
          update_bounding_box(submap_state.width, submap_state.height);
        });
  }

  const int kPaddingPixel = 5;
  const Eigen::Array2i size(
      std::ceil(bounding_box.sizes().x()) + 2 * kPaddingPixel,
      std::ceil(bounding_box.sizes().y()) + 2 * kPaddingPixel);
  const Eigen::Array2f origin(-bounding_box.min().x() + kPaddingPixel,
                              -bounding_box.min().y() + kPaddingPixel);

  auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(kCairoFormat, size.x(), size.y()));
  {
    auto cr =
        ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);
    cairo_paint(cr.get());
    cairo_translate(cr.get(), origin.x(), origin.y());
    CairoDrawEachSubmap(1. / resolution, submaps, cr.get(),
                        [&cr](const SubmapState& submap_state) {
                          cairo_set_source_surface(
                              cr.get(), submap_state.surface.get(), 0., 0.);
                          cairo_paint(cr.get());
                        });
    cairo_surface_flush(surface.get());
  }
  return OccupancyGridState(std::move(surface), origin, size);
}

void WriteOccupancyGridToPgm(const std::string& filename,
                             const double resolution,
                             const OccupancyGridState& grid_state) {
  LOG(INFO) << "Saving map to '" << filename << "'...";
  std::ofstream pgm_file(filename, std::ios::out | std::ios::binary);
  const std::string header = "P5\n# Cartographer map; " +
                             std::to_string(resolution) +
                             " m/pixel\n" + std::to_string(grid_state.size.x()) +
                             " " + std::to_string(grid_state.size.y()) + "\n255\n";
  pgm_file.write(header.data(), header.size());

  const uint32* pixel_data =
      reinterpret_cast<uint32*>(cairo_image_surface_get_data(grid_state.surface.get()));

  for (int y = 0; y < grid_state.size.y(); ++y) {
    for (int x = 0; x < grid_state.size.x(); ++x) {
      const uint32 packed = pixel_data[y * grid_state.size.x() + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value = observed == 0 ? 128 : ::cartographer::common::RoundToInt(
                                                 (1. - color / 255.) * 100.);
      //CHECK_LE(-1, value);
      //CHECK_GE(100, value);
      pgm_file.put(255 - value);
    }
  }

  pgm_file.close();
  CHECK(pgm_file) << "Writing '" << filename << "' failed.";
}
}

  /*
  for (size_t y = 0; y < gridnfo.height; ++y) {
    for (size_t x = 0; x < grid.info.width; ++x) {
      const size_t i = x + (grid.info.height - y - 1) * grid.info.width;
      if (grid.data[i] >= 0 && grid.data[i] <= 100) {
        pgm_file.put((100 - grid.data[i]) * 255 / 100);
      } else {
        // We choose a value between the free and occupied threshold.
        constexpr uint8_t kUnknownValue = 128;
        pgm_file.put(kUnknownValue);
      }
    }
  }
  */
