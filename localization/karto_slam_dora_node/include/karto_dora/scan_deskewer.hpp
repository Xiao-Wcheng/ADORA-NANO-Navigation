#pragma once

#include "karto_dora/odometry_buffer.hpp"
#include "karto_dora/types.hpp"

#include <string>
#include <vector>

namespace karto_dora {

struct DeskewResult {
  Pose2d reference_pose;
  double reference_stamp{0.0};
  double bracket_span{0.0};
  std::vector<Point2d> points;
  std::string drop_reason;
  bool ok() const noexcept { return drop_reason.empty(); }
};

DeskewResult Deskew(const LaserScan &scan, const OdometryBuffer &odometry,
                    const Pose2d &lidar_extrinsic);

}  // namespace karto_dora
