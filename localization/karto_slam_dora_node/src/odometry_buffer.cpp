#include "karto_dora/odometry_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace karto_dora {

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

OdometryBuffer::OdometryBuffer(double retention_seconds)
  : retention_seconds_(retention_seconds)
{
  if (!(retention_seconds_ > 0.0)) throw std::invalid_argument("retention must be positive");
}

void OdometryBuffer::Push(TimedPose2d sample)
{
  if (!std::isfinite(sample.stamp) || !std::isfinite(sample.pose.x) ||
      !std::isfinite(sample.pose.y) || !std::isfinite(sample.pose.yaw)) {
    throw ClockError("non-finite odometry sample");
  }
  if (!samples_.empty() && sample.stamp <= samples_.back().stamp) {
    throw ClockError("odometry timestamp did not increase");
  }
  sample.pose.yaw = NormalizeAngle(sample.pose.yaw);
  samples_.push_back(sample);
  const double oldest = sample.stamp - retention_seconds_;
  while (samples_.size() > 2 && samples_[1].stamp < oldest) samples_.pop_front();
}

BracketStatus OdometryBuffer::Status(double stamp) const
{
  if (!std::isfinite(stamp)) return BracketStatus::ClockError;
  if (samples_.empty()) return BracketStatus::NeedPast;
  if (stamp < samples_.front().stamp) return BracketStatus::TooOld;
  if (stamp > samples_.back().stamp) return BracketStatus::NeedFuture;
  return BracketStatus::Ready;
}

std::optional<Pose2d> OdometryBuffer::Interpolate(double stamp) const
{
  if (Status(stamp) != BracketStatus::Ready) return std::nullopt;
  const auto upper = std::lower_bound(samples_.begin(), samples_.end(), stamp,
    [](const TimedPose2d &sample, double value) { return sample.stamp < value; });
  if (upper == samples_.begin()) return upper->pose;
  if (upper == samples_.end()) return samples_.back().pose;
  if (std::abs(upper->stamp - stamp) < 1.0e-12) return upper->pose;
  const auto lower = std::prev(upper);
  const double ratio = (stamp - lower->stamp) / (upper->stamp - lower->stamp);
  Pose2d result;
  result.x = lower->pose.x + ratio * (upper->pose.x - lower->pose.x);
  result.y = lower->pose.y + ratio * (upper->pose.y - lower->pose.y);
  result.yaw = NormalizeAngle(lower->pose.yaw + ratio * NormalizeAngle(upper->pose.yaw - lower->pose.yaw));
  return result;
}

}  // namespace karto_dora
