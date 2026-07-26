#include "nav2_rpp_port/costmap.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace nav2_rpp_port {
RollingCostmap::RollingCostmap(CostmapConfig c):config_(c){}
void RollingCostmap::reset(){obstacles_.clear();}
void RollingCostmap::insertScanPoints(const std::vector<ObstaclePoint>& p){obstacles_=p;}
double RollingCostmap::clearanceAt(double x,double y) const {
  double d=std::numeric_limits<double>::infinity();
  for(const auto&o:obstacles_) d=std::min(d,std::hypot(o.x-x,o.y-y));
  return d;
}
bool RollingCostmap::trajectoryCollisionFree(const Trajectory&t) const {
  const double limit=config_.robot_radius+config_.safety_margin;
  for(const auto&p:t) if(clearanceAt(p.x,p.y)<=limit) return false;
  return true;
}
}
