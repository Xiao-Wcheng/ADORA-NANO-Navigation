#include "karto_dora/pose_extrapolator.hpp"

#include <cassert>
#include <cmath>

namespace {

bool Near(double a, double b, double tolerance = 1e-9) {
  return std::abs(a - b) <= tolerance;
}

}  // namespace

int main() {
  karto_dora::PoseExtrapolator extrapolator;
  assert(!extrapolator.Predict({0.0, 0.0, 0.0}).has_value());

  extrapolator.ObserveMatch({10.0, 5.0, 1.5707963267948966},
                            {1.0, 2.0, 1.5707963267948966});
  const auto translated = extrapolator.Predict({2.0, 2.0, 1.5707963267948966});
  assert(translated.has_value());
  assert(Near(translated->x, 11.0));
  assert(Near(translated->y, 5.0));
  assert(Near(translated->yaw, 1.5707963267948966));

  const auto rotated = extrapolator.Predict({1.0, 2.0, 3.141592653589793});
  assert(rotated.has_value());
  assert(Near(rotated->x, 10.0));
  assert(Near(rotated->y, 5.0));
  assert(Near(rotated->yaw, 3.141592653589793));

  extrapolator.Reset();
  assert(!extrapolator.Predict({2.0, 2.0, 0.0}).has_value());
}
