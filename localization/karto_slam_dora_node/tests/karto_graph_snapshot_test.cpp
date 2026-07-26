#include "karto_dora/karto_mapper.hpp"
#include "karto_dora/scan_preparer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

karto_dora::PreparedScan MakeScan(karto_dora::Pose2d base_pose, double stamp)
{
  karto_dora::PreparedScan prepared;
  prepared.base_pose = base_pose;
  prepared.stamp = stamp;
  prepared.scan.start_stamp = stamp;
  prepared.scan.angle_min = 0.0;
  prepared.scan.angle_increment = 2.0 * M_PI / 450.0;
  prepared.scan.angle_max = 2.0 * M_PI - prepared.scan.angle_increment;
  prepared.scan.range_min = 0.1;
  prepared.scan.range_max = 12.0;
  prepared.scan.ranges.assign(450, 2.0);
  for (int i = 0; i < 450; ++i) {
    const double angle = i * prepared.scan.angle_increment;
    prepared.base_points.push_back(
        {0.09 + 2.0 * std::cos(angle), 0.06 + 2.0 * std::sin(angle)});
  }
  return prepared;
}

}  // namespace

int main()
{
  auto config = karto_dora::SlamConfig::ReferenceDefaults();
  config.minimum_travel_distance = 0.01;
  karto_dora::KartoMapper mapper(config);
  const karto_dora::Pose2d base_pose{0.5, -0.2, M_PI_2};
  assert(mapper.Process(MakeScan(base_pose, 10.0)).accepted);

  const auto snapshot = mapper.Snapshot();
  assert(snapshot.nodes.size() == 1);
  assert(snapshot.nodes.at(0).stamp == 10.0);
  assert(std::abs(snapshot.nodes.at(0).odometric_pose.x - base_pose.x) < 1e-12);
  assert(std::abs(snapshot.nodes.at(0).odometric_pose.y - base_pose.y) < 1e-12);
  assert(std::abs(snapshot.nodes.at(0).odometric_pose.yaw - base_pose.yaw) < 1e-12);
  assert(snapshot.nodes.at(0).points.size() == 450);
  assert(std::abs(snapshot.nodes.at(0).points.at(0).x - 2.09) < 1e-12);
  assert(std::abs(snapshot.nodes.at(0).points.at(0).y - 0.06) < 1e-12);
  std::cout << "karto_graph_snapshot_test PASS\n";
}
