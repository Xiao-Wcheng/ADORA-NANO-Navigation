#include "karto_dora/scan_deskewer.hpp"

#include <algorithm>
#include <cmath>

namespace karto_dora {
namespace {

Point2d Transform(const Pose2d &pose, const Point2d &point)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

Point2d InverseTransform(const Pose2d &pose, const Point2d &point)
{
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {c * dx + s * dy, -s * dx + c * dy};
}

Pose2d Compose(const Pose2d &base, const Pose2d &offset)
{
  const Point2d translated = Transform(base, {offset.x, offset.y});
  return {translated.x, translated.y, NormalizeAngle(base.yaw + offset.yaw)};
}

}  // namespace

DeskewResult Deskew(const LaserScan &scan, const OdometryBuffer &odometry,
                    const Pose2d &lidar_extrinsic)
{
  DeskewResult result;
  if (scan.ranges.empty()) {
    result.drop_reason = "empty_scan";
    return result;
  }
  result.reference_stamp = 0.5 * (scan.start_stamp + scan.end_stamp);
  const auto reference_base = odometry.Interpolate(result.reference_stamp);
  if (!reference_base) {
    result.drop_reason = "missing_reference_odom_bracket";
    return result;
  }
  result.reference_pose = Compose(*reference_base, lidar_extrinsic);
  result.bracket_span = scan.end_stamp - scan.start_stamp;
  result.points.reserve(scan.ranges.size());

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) continue;
    const double stamp = std::min(scan.end_stamp,
      scan.start_stamp + static_cast<double>(index) * scan.time_increment);
    const auto acquisition_base = odometry.Interpolate(stamp);
    if (!acquisition_base) {
      result.points.clear();
      result.drop_reason = "missing_beam_odom_bracket";
      return result;
    }
    const Pose2d acquisition_lidar = Compose(*acquisition_base, lidar_extrinsic);
    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const Point2d global = Transform(acquisition_lidar, {range * std::cos(angle), range * std::sin(angle)});
    result.points.push_back(InverseTransform(result.reference_pose, global));
  }
  if (result.points.empty()) result.drop_reason = "no_valid_beams";
  return result;
}

}  // namespace karto_dora
