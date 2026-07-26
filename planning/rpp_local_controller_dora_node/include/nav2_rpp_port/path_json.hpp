#pragma once

#include "nav2_rpp_port/types.hpp"

#include <nlohmann/json.hpp>

namespace nav2_rpp_port {

struct PathGridTransform {
  double resolution{0.05};
  int map_height{400};
  double origin_x{-10.0};
  double origin_y{-10.0};
};

Path2D readPathJson(
    const nlohmann::json &message, const PathGridTransform &transform);

}  // namespace nav2_rpp_port
