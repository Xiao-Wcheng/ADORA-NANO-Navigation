#pragma once

#include "karto_dora/types.hpp"

#include <optional>

namespace karto_dora {

class PoseExtrapolator {
 public:
  void ObserveMatch(const Pose2d &corrected_pose, const Pose2d &odometry_pose);
  void Reset() noexcept;
  std::optional<Pose2d> Predict(const Pose2d &odometry_pose) const;

 private:
  std::optional<Pose2d> map_from_odom_;
};

}  // namespace karto_dora
