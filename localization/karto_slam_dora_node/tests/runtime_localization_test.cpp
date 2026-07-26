#include "karto_dora/karto_mapper.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void Require(bool ok, const char *message)
{
  if (!ok) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

karto_dora::PreparedScan MakePreparedScan(double x)
{
  karto_dora::PreparedScan scan;
  scan.stamp = x;
  scan.base_pose = {x, 0.0, 0.0};
  scan.scan.start_stamp = x;
  scan.scan.range_min = 0.05;
  scan.scan.range_max = 8.0;
  scan.scan.angle_min = 0.0;
  scan.scan.angle_increment = 2.0 * M_PI / 450.0;
  scan.scan.angle_max = 2.0 * M_PI - scan.scan.angle_increment;
  scan.scan.ranges.reserve(450);
  scan.base_points.reserve(450);
  for (int beam = 0; beam < 450; ++beam) {
    const double angle = beam * scan.scan.angle_increment;
    const double range = 2.0 + 0.2 * std::sin(beam * 0.04);
    scan.scan.ranges.push_back(range);
    scan.base_points.push_back(
        {0.09 + range * std::cos(angle), 0.06 + range * std::sin(angle)});
  }
  return scan;
}

karto_dora::PoseGraphArchive MakeArchive(std::size_t scan_count)
{
  karto_dora::PoseGraphArchive archive;
  archive.source_tag = "slam_toolbox-2.6.9";
  archive.laser.range_min = 0.05;
  archive.laser.range_max = 8.0;
  archive.laser.angle_min = 0.0;
  archive.laser.angle_increment = 2.0 * M_PI / 450.0;
  archive.laser.angle_max = 2.0 * M_PI - archive.laser.angle_increment;
  archive.laser.extrinsic_x = 0.09;
  archive.laser.extrinsic_y = 0.06;

  double x = 0.0;
  for (std::size_t index = 0; index < scan_count; ++index) {
    if (index != 0) {
      x += index == 13 ? 0.049809 : 0.051;
    }
    const auto prepared = MakePreparedScan(x);
    karto_dora::ScanArchive stored;
    stored.id = static_cast<std::int64_t>(index);
    stored.stamp = static_cast<double>(index) * 0.1;
    stored.odometric_pose = prepared.base_pose;
    stored.optimized_pose = prepared.base_pose;
    stored.points = prepared.base_points;
    archive.scans.push_back(std::move(stored));
  }
  return archive;
}

}  // namespace

int main()
{
  auto config = karto_dora::SlamConfig::ReferenceDefaults();

  {
    karto_dora::KartoMapper mapper(config);
    const auto archive = MakeArchive(20);
    mapper.Restore(archive);
    Require(mapper.scan_count() == archive.scans.size(),
            "closely spaced historical scan was filtered during restore");
  }

  {
    karto_dora::KartoMapper mapper(config);
    auto archive = MakeArchive(1);
    auto scan = MakePreparedScan(0.0);
    mapper.Restore(archive);
    const auto result = mapper.Localize(scan);
    Require(result.accepted, "known scan did not localize");
    Require(result.match_response > 0.35, "known scan confidence too low");
    Require(mapper.scan_count() == 1,
            "localization inserted persistent keyframe");
  }

  {
    karto_dora::KartoMapper mapper(config);
    const auto archive = MakeArchive(20);
    mapper.Restore(archive);
    const auto first = mapper.Localize(MakePreparedScan(1.20));
    Require(first.accepted && !first.skipped,
            "first moved localization scan was not matched");
    const auto nodes_after_match = mapper.graph_node_count();
    for (int index = 1; index <= 30; ++index) {
      const auto stationary = mapper.Localize(MakePreparedScan(1.20 + index * 0.001));
      Require(stationary.skipped && !stationary.accepted,
              "sub-threshold localization scan entered OpenKarto");
    }
    Require(mapper.graph_node_count() == nodes_after_match,
            "skipped localization scans changed the rolling graph");
    Require(mapper.scan_count() == archive.scans.size(),
            "localization changed the persistent map scan count");
  }

  {
    karto_dora::KartoMapper mapper(config);
    const auto archive = MakeArchive(20);
    mapper.Restore(archive);
    mapper.SetInitialPose({0.51, 0.0, 0.0});
    auto scan = MakePreparedScan(0.0);
    const auto relocalized = mapper.Localize(scan);
    Require(relocalized.accepted, "nearby-region relocalization was rejected");
    Require(std::abs(relocalized.corrected_pose.x - 0.51) < 0.20,
            "relocalization ignored the requested initial map pose");
    Require(mapper.scan_count() == archive.scans.size(),
            "relocalization modified the persistent map");
  }

  std::cout << "runtime_localization_test PASS\n";
}
