#pragma once

#include "karto_dora/types.hpp"

#include <deque>
#include <optional>
#include <stdexcept>

namespace karto_dora {

enum class BracketStatus { Ready, NeedPast, NeedFuture, TooOld, ClockError };

class ClockError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

class OdometryBuffer {
public:
  explicit OdometryBuffer(double retention_seconds);
  void Push(TimedPose2d sample);
  BracketStatus Status(double stamp) const;
  std::optional<Pose2d> Interpolate(double stamp) const;
  std::size_t size() const noexcept { return samples_.size(); }

private:
  double retention_seconds_;
  std::deque<TimedPose2d> samples_;
};

double NormalizeAngle(double angle);

}  // namespace karto_dora
