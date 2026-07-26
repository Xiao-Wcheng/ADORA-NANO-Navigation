#include "karto_dora/karto_mapper.hpp"
#include "karto_dora/scan_preparer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

karto_dora::PreparedScan MakeRoomScan(double robot_x, double odom_x)
{
  karto_dora::PreparedScan prepared;
  prepared.stamp = 1.0 + robot_x;
  prepared.base_pose = {odom_x, 0.0, 0.0};
  prepared.scan.start_stamp = prepared.stamp;
  prepared.scan.angle_min = 0.0;
  prepared.scan.angle_increment = 2.0 * M_PI / 450.0;
  prepared.scan.angle_max = 2.0 * M_PI - prepared.scan.angle_increment;
  prepared.scan.range_min = 0.1;
  prepared.scan.range_max = 12.0;
  prepared.scan.ranges.reserve(450);

  const karto_dora::Pose2d extrinsic{0.09, 0.06, 0.0};
  const double laser_x = robot_x + extrinsic.x;
  const double laser_y = extrinsic.y;
  for (int i = 0; i < 450; ++i) {
    const double angle = prepared.scan.angle_min + i * prepared.scan.angle_increment;
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    double range = std::numeric_limits<double>::infinity();
    if (dx > 1e-9) range = std::min(range, (4.0 - laser_x) / dx);
    if (dx < -1e-9) range = std::min(range, (-4.0 - laser_x) / dx);
    if (dy > 1e-9) range = std::min(range, (3.0 - laser_y) / dy);
    if (dy < -1e-9) range = std::min(range, (-3.0 - laser_y) / dy);
    prepared.scan.ranges.push_back(range);
    prepared.base_points.push_back(
        {extrinsic.x + range * dx, extrinsic.y + range * dy});
  }
  return prepared;
}

void Require(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main()
{
  auto config = karto_dora::SlamConfig::ReferenceDefaults();
  config.minimum_travel_distance = 0.01;
  karto_dora::KartoMapper mapper(config);

  const auto first = mapper.Process(MakeRoomScan(0.0, 0.0));
  Require(first.accepted && first.keyframe, "first scan must seed the map");

  const auto second = mapper.Process(MakeRoomScan(0.20, 0.30));
  Require(second.accepted && second.keyframe, "second scan must be accepted");
  Require(second.match_response > 0.50, "scan match response is too weak");
  Require(std::abs(second.corrected_pose.x - 0.20) < 0.06,
          "matcher did not correct translational odometry drift");
  Require(second.constraint_count >= 1, "sequential constraint was not created");

  std::cout << "karto_scan_match_test PASS response=" << second.match_response
            << " corrected_x=" << second.corrected_pose.x << '\n';
}
