#include "nav2_rpp_port/path_handler.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
namespace nav2_rpp_port {
PathHandler::PathHandler(PathHandlerConfig c) : config_(c) {
  if (c.max_search_distance <= 0 || c.max_local_path_length <= 0 ||
      c.minimum_pose_separation < 0) throw std::invalid_argument("invalid path handler config");
}
PathHandlerResult PathHandler::transformAndPrune(
  const Path2D & path, const Pose2D & robot) const {
  PathHandlerResult out;
  if (path.empty() || !finite(robot)) return out;
  for (const auto & p : path) if (!finite(p)) return out;
  double integrated = 0, best = std::numeric_limits<double>::infinity();
  std::size_t closest = 0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (i > 0) integrated += std::hypot(path[i].x-path[i-1].x, path[i].y-path[i-1].y);
    if (integrated > config_.max_search_distance) break;
    const double d = std::hypot(path[i].x-robot.x, path[i].y-robot.y);
    if (d < best) { best = d; closest = i; }
  }
  const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
  double local_length = 0;
  for (std::size_t i = closest; i < path.size(); ++i) {
    if (i > closest) {
      const double segment = std::hypot(path[i].x-path[i-1].x, path[i].y-path[i-1].y);
      if (segment < config_.minimum_pose_separation) continue;
      local_length += segment;
      if (local_length > config_.max_local_path_length && !out.local_path.empty()) break;
    }
    const double dx = path[i].x-robot.x, dy = path[i].y-robot.y;
    out.local_path.push_back({c*dx+s*dy, -s*dx+c*dy, normalizeAngle(path[i].yaw-robot.yaw)});
  }
  out.valid = !out.local_path.empty();
  out.reason = out.valid ? "ok" : "empty_local_path";
  out.closest_index = closest;
  out.pruned_count = closest;
  return out;
}
}
