#include "nav2_velocity_smoother_port/velocity_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav2_velocity_smoother_port
{
namespace
{

double smoothAxis(
  double current, double target, double dt, double maximum, double minimum,
  double accel, double decel, double deadband)
{
  target = std::clamp(target, minimum, maximum);
  const bool accelerating = std::abs(target) > std::abs(current) &&
    (current == 0.0 || std::signbit(target) == std::signbit(current));
  const double limit = (accelerating ? std::abs(accel) : std::abs(decel)) * dt;
  const double delta = std::clamp(target - current, -limit, limit);
  const double value = current + delta;
  return std::abs(value) < std::max(0.0, deadband) ? 0.0 : value;
}

}  // namespace

VelocitySmoother::VelocitySmoother(Config config) : config_(config)
{
  if (!(config_.frequency > 0.0) || config_.velocity_timeout_ms < 0) {
    throw std::invalid_argument("invalid velocity smoother configuration");
  }
}

void VelocitySmoother::setTarget(const Twist & target, int64_t now_ms)
{
  target_ = target;
  last_target_ms_ = now_ms;
}

Twist VelocitySmoother::update(int64_t now_ms)
{
  if (last_target_ms_ < 0 || now_ms - last_target_ms_ > config_.velocity_timeout_ms) {
    reset();
    return current_;
  }
  const double dt = 1.0 / config_.frequency;
  current_.vx = smoothAxis(
    current_.vx, target_.vx, dt, config_.max_velocity[0], config_.min_velocity[0],
    config_.max_accel[0], config_.max_decel[0], config_.deadband[0]);
  current_.vy = smoothAxis(
    current_.vy, target_.vy, dt, config_.max_velocity[1], config_.min_velocity[1],
    config_.max_accel[1], config_.max_decel[1], config_.deadband[1]);
  current_.wz = smoothAxis(
    current_.wz, target_.wz, dt, config_.max_velocity[2], config_.min_velocity[2],
    config_.max_accel[2], config_.max_decel[2], config_.deadband[2]);
  return current_;
}

void VelocitySmoother::reset()
{
  target_ = {};
  current_ = {};
  last_target_ms_ = -1;
}

}  // namespace nav2_velocity_smoother_port
