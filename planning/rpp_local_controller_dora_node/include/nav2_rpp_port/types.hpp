#pragma once
#include <cmath>
#include <vector>
namespace nav2_rpp_port {
struct Pose2D { double x{0}, y{0}, yaw{0}; };
struct Twist2D { double vx{0}, vy{0}, wz{0}; };
using Path2D = std::vector<Pose2D>;
using Trajectory = std::vector<Pose2D>;
struct ObstaclePoint { double x{0}, y{0}; };
inline double normalizeAngle(double a) {
  constexpr double pi = 3.14159265358979323846;
  while (a > pi) a -= 2 * pi;
  while (a <= -pi) a += 2 * pi;
  return a;
}
inline bool finite(const Pose2D & p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
inline bool finite(const Twist2D & t) {
  return std::isfinite(t.vx) && std::isfinite(t.vy) && std::isfinite(t.wz);
}
}
