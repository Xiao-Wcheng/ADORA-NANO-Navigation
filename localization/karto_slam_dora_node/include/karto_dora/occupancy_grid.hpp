#pragma once

#include "karto_dora/archive.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace karto_dora {

struct GridConfig {
  double resolution{0.05};
  double crop_margin{0.5};
  double free_threshold{0.25};
  double occupied_threshold{0.65};
};

struct OccupancyGrid {
  std::size_t width{0};
  std::size_t height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<std::int8_t> cells;

  std::int8_t At(std::size_t x, std::size_t y) const;
  std::int8_t AtMetric(double x, double y) const;
};

struct SavedMap {
  std::filesystem::path pgm;
  std::filesystem::path yaml;
  std::filesystem::path metadata;
};

OccupancyGrid BuildOccupancyGrid(const PoseGraphArchive &graph,
                                 const GridConfig &config);
SavedMap SaveMapAtomic(const std::filesystem::path &prefix,
                       const OccupancyGrid &grid,
                       const PoseGraphArchive &graph);

}  // namespace karto_dora
