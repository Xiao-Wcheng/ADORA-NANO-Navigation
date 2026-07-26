#include "nav2_simple_smoother.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace fs = std::filesystem;

namespace {

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

MapLoader LoadOpenMap(const fs::path &root) {
  const fs::path pgm = root / "open.pgm";
  const fs::path yaml = root / "open.yaml";
  std::vector<unsigned char> pixels(80 * 80, 254);
  std::ofstream image(pgm, std::ios::binary);
  image << "P5\n80 80\n255\n";
  image.write(reinterpret_cast<const char *>(pixels.data()),
              static_cast<std::streamsize>(pixels.size()));
  image.close();
  std::ofstream metadata(yaml);
  metadata << "image: open.pgm\nresolution: 0.05\n"
           << "origin: [0.0, 0.0, 0.0]\nnegate: 0\n"
           << "occupied_thresh: 0.65\nfree_thresh: 0.196\n";
  metadata.close();
  MapLoader map;
  Require(map.loadMap(yaml.string()), "test map did not load");
  return map;
}

double Roughness(const std::vector<GridPathPoint> &path) {
  double value = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const double ddx = path[i - 1].x - 2.0 * path[i].x + path[i + 1].x;
    const double ddy = path[i - 1].y - 2.0 * path[i].y + path[i + 1].y;
    value += std::hypot(ddx, ddy);
  }
  return value;
}

}  // namespace

int main() {
  const fs::path root = fs::temp_directory_path() / "nav2_simple_smoother_test";
  fs::remove_all(root);
  fs::create_directories(root);
  auto map = LoadOpenMap(root);

  std::vector<Point> raw;
  raw.reserve(52);
  for (int i = 0; i < 26; ++i) raw.emplace_back(10 + i, 10);
  for (int i = 1; i <= 26; ++i) raw.emplace_back(35, 10 + i);

  std::vector<GridPathPoint> original;
  original.reserve(raw.size());
  for (const auto &point : raw) original.push_back({double(point.x), double(point.y)});

  Nav2SimpleSmoother smoother(map);
  const auto smooth = smoother.smooth(raw);

  Require(smooth.size() == raw.size(), "smoother changed path density");
  Require(smooth.front() == original.front(), "smoother changed first endpoint");
  Require(smooth.back() == original.back(), "smoother changed final endpoint");
  Require(Roughness(smooth) < Roughness(original), "smoother did not reduce roughness");
  for (const auto &point : smooth) {
    Require(map.isValid(static_cast<int>(std::lround(point.x)),
                        static_cast<int>(std::lround(point.y))),
            "smoother produced an out-of-map pose");
    Require(!map.isOccupied(static_cast<int>(std::lround(point.x)),
                            static_cast<int>(std::lround(point.y))),
            "smoother produced a colliding pose");
  }

  fs::remove_all(root);
  std::cout << "nav2_simple_smoother_test PASS\n";
  return 0;
}
