#include "karto_dora/scan_preparer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  karto_dora::OdometryBuffer odometry(3.0);
  odometry.Push({9.9, {0.0, 0.0, 0.0}});
  odometry.Push({10.1, {0.0, 0.0, 0.0}});

  karto_dora::LaserScan scan;
  scan.start_stamp = 10.0;
  scan.end_stamp = 10.1;
  scan.angle_min = 0.0;
  scan.angle_max = 2.0 * M_PI - 2.0 * M_PI / 450.0;
  scan.angle_increment = 2.0 * M_PI / 450.0;
  scan.scan_time = 0.1;
  scan.time_increment = 0.1 / 450.0;
  scan.range_min = 0.1;
  scan.range_max = 12.0;
  scan.ranges.assign(450, 0.0);
  scan.ranges.at(0) = 1.0;

  const auto prepared = karto_dora::PrepareFixedBeamScan(
      scan, odometry, {0.09, 0.06, 0.0}, 450);
  assert(prepared.ok());
  assert(prepared.scan.ranges.size() == 450);
  assert(prepared.stamp == 10.0);
  assert(prepared.base_pose.x == 0.0 && prepared.base_pose.y == 0.0);
  assert(prepared.base_points.size() == 1);
  assert(std::abs(prepared.base_points.at(0).x - 1.09) < 1e-9);
  assert(std::abs(prepared.base_points.at(0).y - 0.06) < 1e-9);

  scan.ranges.pop_back();
  const auto wrong_count = karto_dora::PrepareFixedBeamScan(
      scan, odometry, {0.09, 0.06, 0.0}, 450);
  assert(!wrong_count.ok());
  assert(wrong_count.drop_reason == "unexpected_beam_count");
  std::cout << "scan_preparer_test PASS\n";
}
