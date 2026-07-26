#pragma once
#include "nav2_rpp_port/types.hpp"
namespace nav2_rpp_port {
struct RotationResult { bool active{false}, valid{true}; double error{0}; Twist2D command{}; };
class RotationShim {
public:
  RotationResult compute(const Path2D &, bool clear);
  void reset(){active_=false;}
private: bool active_{false};
};
}
