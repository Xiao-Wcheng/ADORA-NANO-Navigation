#include "karto_dora/karto_mapper.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {

karto_dora::PreparedScan MakeRectangleScan(const karto_dora::Pose2d &pose,
                                           double stamp)
{
  constexpr std::size_t kBeams = 450;
  const karto_dora::Pose2d extrinsic{0.09, 0.06, 0.0};
  karto_dora::PreparedScan prepared;
  prepared.base_pose = pose;
  prepared.stamp = stamp;
  prepared.scan.start_stamp = stamp;
  prepared.scan.angle_min = 0.0;
  prepared.scan.angle_increment = 2.0 * M_PI / static_cast<double>(kBeams);
  prepared.scan.angle_max = 2.0 * M_PI - prepared.scan.angle_increment;
  prepared.scan.range_min = 0.05;
  prepared.scan.range_max = 12.0;

  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double laser_world_x = pose.x + c * extrinsic.x - s * extrinsic.y;
  const double laser_world_y = pose.y + s * extrinsic.x + c * extrinsic.y;
  for (std::size_t index = 0; index < kBeams; ++index) {
    const double local_angle = index * prepared.scan.angle_increment;
    const double world_angle = pose.yaw + local_angle;
    const double dx = std::cos(world_angle);
    const double dy = std::sin(world_angle);
    double range = std::numeric_limits<double>::infinity();
    if (dx > 1e-10) range = std::min(range, (4.0 - laser_world_x) / dx);
    if (dx < -1e-10) range = std::min(range, (-4.0 - laser_world_x) / dx);
    if (dy > 1e-10) range = std::min(range, (3.0 - laser_world_y) / dy);
    if (dy < -1e-10) range = std::min(range, (-3.0 - laser_world_y) / dy);
    assert(std::isfinite(range) && range > prepared.scan.range_min);
    prepared.scan.ranges.push_back(range);
    prepared.base_points.push_back(
        {extrinsic.x + range * std::cos(local_angle),
         extrinsic.y + range * std::sin(local_angle)});
  }
  return prepared;
}

}  // namespace

int main()
{
  auto config = karto_dora::SlamConfig::ReferenceDefaults();
  config.minimum_travel_distance = 0.01;
  config.minimum_travel_heading = 0.01;
  karto_dora::KartoMapper mapper(config);
  std::vector<karto_dora::Pose2d> route;
  for (int i = 0; i <= 20; ++i) route.push_back({-2.5 + 0.25 * i, -1.5, 0.0});
  for (int i = 1; i <= 12; ++i) route.push_back({2.5, -1.5 + 0.25 * i, M_PI_2});
  for (int i = 1; i <= 20; ++i) route.push_back({2.5 - 0.25 * i, 1.5, M_PI});
  for (int i = 1; i <= 12; ++i) route.push_back({-2.5, 1.5 - 0.25 * i, -M_PI_2});
  for (std::size_t index = 0; index < route.size(); ++index) {
    auto scan = MakeRectangleScan(route[index], 10.0 + 0.1 * index);
    const auto result = mapper.Process(scan);
    assert(result.accepted);
  }

  const auto snapshot = mapper.Snapshot();
  assert(snapshot.nodes.size() == route.size());
  assert(snapshot.constraints.size() >= snapshot.nodes.size() - 1);
  assert(snapshot.constraints.size() == snapshot.sequential_constraints +
         snapshot.near_chain_constraints + snapshot.loop_closures);
  std::cerr << "loop diagnostics: candidates=" << snapshot.loop_candidate_checks
            << " fine=" << snapshot.loop_fine_checks
            << " rejected=" << snapshot.loop_rejections
            << " sequential=" << snapshot.sequential_constraints
            << " near=" << snapshot.near_chain_constraints
            << " loops=" << snapshot.loop_closures << '\n';
  assert(snapshot.near_chain_constraints > 0);

  std::array<std::size_t, 4> wall_hits{};
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -min_x;
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -min_y;
  std::size_t aligned_points = 0;
  for (const auto &node : snapshot.nodes) {
    assert(node.points.size() == 450);
    const double c = std::cos(node.optimized_pose.yaw);
    const double s = std::sin(node.optimized_pose.yaw);
    for (const auto &point : node.points) {
      const double x = node.optimized_pose.x + c * point.x - s * point.y;
      const double y = node.optimized_pose.y + s * point.x + c * point.y;
      min_x = std::min(min_x, x); max_x = std::max(max_x, x);
      min_y = std::min(min_y, y); max_y = std::max(max_y, y);
      const std::array<double, 4> errors = {
          std::abs(x + 4.0), std::abs(x - 4.0),
          std::abs(y + 3.0), std::abs(y - 3.0)};
      const auto wall = std::min_element(errors.begin(), errors.end());
      if (*wall < 0.15) {
        ++aligned_points;
        ++wall_hits[static_cast<std::size_t>(wall - errors.begin())];
      }
    }
  }
  assert(aligned_points > snapshot.nodes.size() * 450 * 9 / 10);
  for (const auto hits : wall_hits) assert(hits > 100);
  std::cerr << "optimized rectangle: width=" << max_x - min_x
            << " height=" << max_y - min_y
            << " initial_cost=" << snapshot.solver.initial_cost
            << " final_cost=" << snapshot.solver.final_cost
            << " iterations=" << snapshot.solver.iterations << '\n';
  assert(max_x - min_x > 7.5 && max_x - min_x < 8.5);
  assert(max_y - min_y > 5.5 && max_y - min_y < 6.5);
  std::cout << "rectangle_mapping_test PASS width=" << max_x - min_x
            << " height=" << max_y - min_y
            << " constraints=" << snapshot.constraints.size() << '\n';
}
