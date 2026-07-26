#include "karto_dora/scan_deskewer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  karto_dora::OdometryBuffer buffer(3.0);
  buffer.Push({0.0, {0.0, 0.0, 0.0}});
  buffer.Push({1.0, {1.0, 0.0, 0.0}});

  karto_dora::LaserScan scan;
  scan.start_stamp = 0.0;
  scan.end_stamp = 1.0;
  scan.time_increment = 0.5;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.ranges = {2.0, 1.5, 1.0};

  const auto result = karto_dora::Deskew(scan, buffer, {0.0, 0.0, 0.0});
  assert(result.ok());
  assert(result.points.size() == 3);
  for (const auto &point : result.points) {
    assert(std::abs(point.x - 1.5) < 1e-9);
    assert(std::abs(point.y) < 1e-9);
  }
  assert(std::abs(result.reference_stamp - 0.5) < 1e-9);
  std::cout << "scan_deskewer_test PASS\n";
}
