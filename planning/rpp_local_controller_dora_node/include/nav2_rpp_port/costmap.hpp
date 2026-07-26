#pragma once
#include "nav2_rpp_port/types.hpp"
#include <vector>
namespace nav2_rpp_port {
struct CostmapConfig { double size_x{2}, size_y{2}, resolution{0.05}, robot_radius{0.17}, safety_margin{0.04}; };
class RollingCostmap {
public:
  explicit RollingCostmap(CostmapConfig config);
  void reset();
  void insertScanPoints(const std::vector<ObstaclePoint> & points);
  double clearanceAt(double x, double y) const;
  bool trajectoryCollisionFree(const Trajectory & trajectory) const;
private:
  CostmapConfig config_;
  std::vector<ObstaclePoint> obstacles_;
};
}
