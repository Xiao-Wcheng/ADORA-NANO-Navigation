#include "karto_dora/odometry_buffer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  karto_dora::OdometryBuffer buffer(3.0);
  buffer.Push({10.0, {0.0, 0.0, 3.124139361}});
  buffer.Push({10.1, {1.0, 2.0, -3.124139361}});
  assert(buffer.Status(10.05) == karto_dora::BracketStatus::Ready);
  const auto pose = buffer.Interpolate(10.05);
  assert(pose.has_value());
  assert(std::abs(pose->x - 0.5) < 1e-9);
  assert(std::abs(pose->y - 1.0) < 1e-9);
  assert(std::abs(std::abs(pose->yaw) - 3.141592653589793) < 1e-6);
  assert(buffer.Status(10.2) == karto_dora::BracketStatus::NeedFuture);

  bool rejected = false;
  try {
    buffer.Push({10.05, {0.0, 0.0, 0.0}});
  } catch (const karto_dora::ClockError &) {
    rejected = true;
  }
  assert(rejected);
  std::cout << "odometry_buffer_test PASS\n";
}
