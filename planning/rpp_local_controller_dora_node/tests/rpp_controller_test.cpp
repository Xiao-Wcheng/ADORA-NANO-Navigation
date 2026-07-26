#include "nav2_rpp_port/rpp_controller.hpp"
#include <cassert>
#include <cmath>
int main() {
  using namespace nav2_rpp_port;
  RollingCostmap clear({2,2,0.05,0.17,0.04});
  RppController controller({});
  auto r = controller.compute({{0,0,0},{1,0,0}}, {}, clear);
  assert(r.valid && r.command.vx > 0 && r.command.vy == 0 && std::abs(r.command.wz)<1e-9);
  r = controller.compute({{0,0,0},{0.4,0.2,0}}, {}, clear);
  assert(r.valid && r.command.wz > 0 && r.command.vy == 0);
  auto right = controller.compute({{0,0,0},{0.4,-0.2,0}}, {}, clear);
  assert(right.valid && right.command.wz < 0);
  assert(r.command.vx < 0.04);
  auto near = controller.compute({{0,0,0},{0.1,0,0}}, {}, clear);
  assert(near.valid && near.command.vx < 0.04);
  RollingCostmap blocked({2,2,0.05,0.17,0.04});
  blocked.insertScanPoints({{0.24,0}});
  auto collision = controller.compute({{0,0,0},{1,0,0}}, {}, blocked);
  assert(!collision.valid && collision.reason == "collision_ahead");
}
