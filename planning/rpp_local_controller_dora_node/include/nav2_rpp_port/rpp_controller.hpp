#pragma once
#include "nav2_rpp_port/costmap.hpp"
#include <string>
namespace nav2_rpp_port {
struct RppConfig {
  double desired_linear_velocity{0.04};
  double min_lookahead{0.20}, max_lookahead{0.45}, lookahead_time{1.5};
  double regulated_min_radius{0.80}, regulated_min_speed{0.012};
  double approach_scaling_distance{0.35}, min_approach_speed{0.008};
  double clearance_scaling_distance{0.45};
  double max_angular_speed{0.20}, collision_horizon{1.5}, collision_dt{0.05};
};
struct RppResult {
  bool valid{false};
  std::string reason{"invalid_path"};
  Twist2D command{};
  double lookahead_distance{0}, curvature{0}, regulation_factor{0};
  Pose2D carrot{};
  bool collision_free{false};
};
class RppController {
public:
  explicit RppController(RppConfig config);
  RppResult compute(const Path2D &, const Twist2D &, const RollingCostmap &) const;
private:RppConfig config_;
};
}
