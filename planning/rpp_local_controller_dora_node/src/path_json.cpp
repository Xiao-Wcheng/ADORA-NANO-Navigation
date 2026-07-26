#include "nav2_rpp_port/path_json.hpp"

#include <cmath>

namespace nav2_rpp_port {

Path2D readPathJson(
    const nlohmann::json &message, const PathGridTransform &transform) {
  Path2D path;
  if (!message.value("path_found", false) ||
      !message.contains("waypoints") ||
      !message["waypoints"].is_array()) {
    return path;
  }

  for (const auto &waypoint : message["waypoints"]) {
    const double grid_x = waypoint.value("x", 0.0);
    const double image_y = waypoint.value("y", 0.0);
    if (!std::isfinite(grid_x) || !std::isfinite(image_y)) return {};
    const double grid_y =
        static_cast<double>(transform.map_height - 1) - image_y;
    path.push_back(
        {transform.origin_x + (grid_x + 0.5) * transform.resolution,
         transform.origin_y + (grid_y + 0.5) * transform.resolution, 0.0});
  }

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    path[i].yaw =
        std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  }
  if (path.size() > 1) path.back().yaw = path[path.size() - 2].yaw;
  return path;
}

}  // namespace nav2_rpp_port
