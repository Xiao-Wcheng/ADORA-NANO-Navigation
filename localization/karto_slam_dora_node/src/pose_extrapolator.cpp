#include "karto_dora/pose_extrapolator.hpp"

#include <cmath>

namespace karto_dora {
namespace {

double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2d Compose(const Pose2d &a, const Pose2d &b) {
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y,
          a.y + s * b.x + c * b.y,
          NormalizeAngle(a.yaw + b.yaw)};
}

Pose2d Inverse(const Pose2d &pose) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {-c * pose.x - s * pose.y,
           s * pose.x - c * pose.y,
          -pose.yaw};
}

}  // namespace

void PoseExtrapolator::ObserveMatch(const Pose2d &corrected_pose,
                                    const Pose2d &odometry_pose) {
  map_from_odom_ = Compose(corrected_pose, Inverse(odometry_pose));
}

void PoseExtrapolator::Reset() noexcept {
  map_from_odom_.reset();
}

std::optional<Pose2d> PoseExtrapolator::Predict(const Pose2d &odometry_pose) const {
  if (!map_from_odom_) return std::nullopt;
  return Compose(*map_from_odom_, odometry_pose);
}

}  // namespace karto_dora
