#include "supervisor_progress.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

SupervisorProgress::SupervisorProgress(SupervisorProgressConfig config)
  : config_(config)
{
  if (config_.required_movement_radius_m < 0.0 ||
    config_.required_movement_angle_rad < 0.0 || config_.timeout_ms <= 0)
  {
    throw std::invalid_argument("invalid supervisor progress configuration");
  }
}

void SupervisorProgress::reset(const SupervisorProgressPose & pose, int64_t now_ms)
{
  initialized_ = std::isfinite(pose.x) && std::isfinite(pose.y) &&
    std::isfinite(pose.yaw);
  baseline_ = initialized_ ? pose : SupervisorProgressPose{};
  last_progress_ms_ = initialized_ ? now_ms : 0;
}

void SupervisorProgress::clear()
{
  initialized_ = false;
  baseline_ = {};
  last_progress_ms_ = 0;
}

bool SupervisorProgress::stalled(const SupervisorProgressPose & pose, int64_t now_ms)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw))
  {
    return false;
  }
  if (!initialized_)
  {
    reset(pose, now_ms);
    return false;
  }
  const double movement = std::hypot(pose.x - baseline_.x, pose.y - baseline_.y);
  const double angle = std::abs(std::atan2(
    std::sin(pose.yaw - baseline_.yaw), std::cos(pose.yaw - baseline_.yaw)));
  if (movement >= config_.required_movement_radius_m ||
    angle >= config_.required_movement_angle_rad)
  {
    baseline_ = pose;
    last_progress_ms_ = now_ms;
    return false;
  }
  return now_ms - last_progress_ms_ > config_.timeout_ms;
}

bool SupervisorProgress::initialized() const
{
  return initialized_;
}

int64_t SupervisorProgress::lastProgressMs() const
{
  return last_progress_ms_;
}
