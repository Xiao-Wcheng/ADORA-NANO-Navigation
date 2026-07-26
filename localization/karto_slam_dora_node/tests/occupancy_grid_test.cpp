#include "karto_dora/occupancy_grid.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

namespace {
void Require(bool value, const char *message)
{
  if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
std::string Read(const fs::path &path)
{
  std::ifstream in(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), {});
}
}  // namespace

int main()
{
  karto_dora::PoseGraphArchive graph;
  graph.source_tag = "slam_toolbox-2.6.9";
  graph.config_hash = "map-test";
  // From the origin, endpoints at +x and +y must mark rays free and endpoints occupied.
  graph.scans.push_back({1, 1.0, {}, {}, {{2.0, 0.0}, {0.0, 1.0}}});
  karto_dora::GridConfig config;
  config.resolution = 0.5;
  config.crop_margin = 0.5;
  config.free_threshold = 0.25;
  config.occupied_threshold = 0.65;

  const auto grid = karto_dora::BuildOccupancyGrid(graph, config);
  Require(grid.width == 6 && grid.height == 4, "metric bounds or crop margin incorrect");
  Require(grid.origin_x == -0.5 && grid.origin_y == -0.5, "map origin incorrect");
  Require(grid.AtMetric(0.5, 0.0) == 0, "ray cells must be free");
  Require(grid.AtMetric(2.0, 0.0) == 100, "ray endpoint must be occupied");
  Require(grid.AtMetric(0.0, 1.0) == 100, "vertical endpoint must be occupied");
  Require(grid.AtMetric(-0.5, 1.0) == -1, "unobserved cell must remain unknown");

  const fs::path root = fs::temp_directory_path() / "karto_dora_grid_test";
  fs::remove_all(root);
  fs::create_directories(root);
  const auto saved = karto_dora::SaveMapAtomic(root / "room", grid, graph);
  Require(fs::exists(saved.pgm) && fs::exists(saved.yaml) && fs::exists(saved.metadata),
          "one or more standard map files missing");
  const std::string yaml = Read(saved.yaml);
  Require(yaml.find("image: room.pgm") != std::string::npos, "YAML image is not relative");
  Require(yaml.find("resolution: 0.5") != std::string::npos, "YAML resolution missing");
  Require(yaml.find("origin: [-0.5, -0.5, 0.0]") != std::string::npos,
          "YAML origin missing");
  const std::string pgm = Read(saved.pgm);
  const auto pixels = pgm.substr(pgm.find("255\n") + 4);
  Require(pixels.size() == grid.width * grid.height, "PGM dimensions mismatch");
  // PGM begins at the map's highest y row; occupied is black (0), unknown 205.
  Require(static_cast<unsigned char>(pixels[1]) == 0, "PGM vertical orientation incorrect");
  Require(static_cast<unsigned char>(pixels[0]) == 205, "unknown PGM encoding incorrect");
  Require(!fs::exists((root / "room.pgm.tmp")), "temporary output was left behind");
  fs::remove_all(root);
  std::cout << "occupancy_grid_test PASS " << grid.width << 'x' << grid.height << '\n';
}
