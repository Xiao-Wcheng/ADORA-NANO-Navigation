#pragma once

#include "karto_dora/odometry_buffer.hpp"
#include "karto_dora/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace karto_dora {

struct PreparedScan {
  LaserScan scan;
  Pose2d base_pose;
  std::vector<Point2d> base_points;
  double stamp{0.0};
  std::string drop_reason;

  bool ok() const noexcept { return drop_reason.empty(); }
};

PreparedScan PrepareFixedBeamScan(const LaserScan &scan,
                                  const OdometryBuffer &odometry,
                                  const Pose2d &lidar_extrinsic,
                                  std::size_t expected_beams);

}  // namespace karto_dora
