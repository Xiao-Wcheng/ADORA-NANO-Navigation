#include "karto_dora/scan_preparer.hpp"

#include <cmath>

namespace karto_dora {

PreparedScan PrepareFixedBeamScan(const LaserScan &scan,
                                  const OdometryBuffer &odometry,
                                  const Pose2d &lidar_extrinsic,
                                  std::size_t expected_beams)
{
  PreparedScan result;
  result.scan = scan;
  result.stamp = scan.start_stamp;

  if (scan.ranges.size() != expected_beams) {
    result.drop_reason = "unexpected_beam_count";
    return result;
  }
  if (!std::isfinite(scan.angle_increment) || scan.angle_increment <= 0.0) {
    result.drop_reason = "invalid_angle_increment";
    return result;
  }
  if (!std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0.0 || scan.range_max <= scan.range_min) {
    result.drop_reason = "invalid_range_limits";
    return result;
  }

  const auto base_pose = odometry.Interpolate(scan.start_stamp);
  if (!base_pose) {
    result.drop_reason = "missing_scan_start_odom";
    return result;
  }
  result.base_pose = *base_pose;

  const double cos_offset = std::cos(lidar_extrinsic.yaw);
  const double sin_offset = std::sin(lidar_extrinsic.yaw);
  result.base_points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const double laser_x = range * std::cos(angle);
    const double laser_y = range * std::sin(angle);
    result.base_points.push_back({
        lidar_extrinsic.x + cos_offset * laser_x - sin_offset * laser_y,
        lidar_extrinsic.y + sin_offset * laser_x + cos_offset * laser_y});
  }
  return result;
}

}  // namespace karto_dora
