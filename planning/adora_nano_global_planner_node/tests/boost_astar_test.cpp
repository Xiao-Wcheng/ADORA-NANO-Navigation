#include "astar.h"
#include "map_loader.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

fs::path WriteMap(const fs::path &root, const std::string &name,
                  int width, int height, const std::vector<unsigned char> &pixels) {
  const fs::path pgm = root / (name + ".pgm");
  const fs::path yaml = root / (name + ".yaml");
  {
    std::ofstream output(pgm, std::ios::binary);
    output << "P5\n" << width << ' ' << height << "\n255\n";
    output.write(reinterpret_cast<const char *>(pixels.data()),
                 static_cast<std::streamsize>(pixels.size()));
  }
  {
    std::ofstream output(yaml);
    output << "image: " << pgm.filename().string() << "\n"
           << "resolution: 0.05\n"
           << "origin: [0.0, 0.0, 0.0]\n"
           << "negate: 0\n"
           << "occupied_thresh: 0.65\n"
           << "free_thresh: 0.196\n";
  }
  return yaml;
}

MapLoader Load(const fs::path &yaml) {
  MapLoader map;
  Require(map.loadMap(yaml.string()), "test map did not load");
  return map;
}

}  // namespace

int main() {
  const fs::path root = fs::temp_directory_path() / "boost_astar_test";
  fs::remove_all(root);
  fs::create_directories(root);

  {
    auto map = Load(WriteMap(root, "open", 5, 5, std::vector<unsigned char>(25, 254)));
    AStar planner(map);
    const auto path = planner.findPath({0, 0}, {4, 3});
    Require(planner.pathFound() && path.front() == Point(0, 0) && path.back() == Point(4, 3),
            "open-grid path endpoints are incorrect");
    Require(std::abs(planner.getPathLength() - (3.0 * std::sqrt(2.0) + 1.0)) < 1e-3,
            "open-grid path is not octile-optimal");
  }

  {
    // Start and goal touch diagonally, but both cardinal exits are occupied.
    auto map = Load(WriteMap(root, "blocked_corner", 2, 2, {254, 0, 0, 254}));
    AStar planner(map);
    Require(planner.findPath({0, 0}, {1, 1}).empty(),
            "planner cut diagonally through an occupied corner");
  }

  {
    auto map = Load(WriteMap(root, "occupied_endpoint", 3, 3,
                             {0, 254, 254, 254, 254, 254, 254, 254, 254}));
    AStar planner(map);
    Require(planner.findPath({0, 0}, {2, 2}).empty(),
            "planner accepted an occupied start");
  }

  {
    auto map = Load(WriteMap(root, "disconnected", 3, 3,
                             {254, 254, 254, 0, 0, 0, 254, 254, 254}));
    AStar planner(map);
    Require(planner.findPath({0, 0}, {2, 2}).empty(),
            "planner crossed a disconnected obstacle wall");
  }

  fs::remove_all(root);
  std::cout << "boost_astar_test PASS\n";
}
