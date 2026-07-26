#pragma once
#include "nav2_rpp_port/types.hpp"
#include <cstddef>
#include <string>
namespace nav2_rpp_port {
struct PathHandlerConfig {
  double max_search_distance{3.0};
  double max_local_path_length{2.0};
  double minimum_pose_separation{0.01};
};
struct PathHandlerResult {
  bool valid{false};
  std::string reason{"invalid_path"};
  std::size_t closest_index{0};
  std::size_t pruned_count{0};
  Path2D local_path{};
};
class PathHandler {
public:
  explicit PathHandler(PathHandlerConfig config);
  PathHandlerResult transformAndPrune(const Path2D & path, const Pose2D & robot) const;
private:
  PathHandlerConfig config_;
};
}
