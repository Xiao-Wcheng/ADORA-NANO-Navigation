#include "nav2_velocity_smoother_port/velocity_smoother.hpp"

#include <cassert>
#include <cmath>

int main()
{
  using namespace nav2_velocity_smoother_port;
  Config config;
  config.frequency = 20.0;
  config.deadband = {0.005, 0.005, 0.02};
  VelocitySmoother smoother(config);

  smoother.setTarget({0.003, -0.004, 0.01}, 0);
  auto output = smoother.update(0);
  assert(output.vx == 0.0 && output.vy == 0.0 && output.wz == 0.0);

  smoother.setTarget({0.04, 0.04, 0.20}, 10);
  output = smoother.update(10);
  assert(std::abs(output.vx - 0.005) < 1e-12);
  assert(std::abs(output.vy - 0.005) < 1e-12);
  assert(std::abs(output.wz - 0.025) < 1e-12);

  output = smoother.update(60);
  assert(std::abs(output.vx - 0.010) < 1e-12);

  smoother.setTarget({}, 70);
  output = smoother.update(70);
  assert(std::abs(output.vx - 0.005) < 1e-12);
  output = smoother.update(120);
  assert(output.vx == 0.0 && output.vy == 0.0 && output.wz == 0.0);

  smoother.setTarget({0.04, 0.0, 0.0}, 200);
  smoother.update(200);
  output = smoother.update(600);
  assert(output.vx == 0.0 && output.vy == 0.0 && output.wz == 0.0);
  return 0;
}
