#pragma once

#include <cstdint>

struct SupervisorProgressConfig
{
  double required_movement_radius_m{0.05};
  double required_movement_angle_rad{0.10};
  int64_t timeout_ms{10000};
};

struct SupervisorProgressPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class SupervisorProgress
{
public:
  explicit SupervisorProgress(SupervisorProgressConfig config);

  void reset(const SupervisorProgressPose & pose, int64_t now_ms);
  void clear();
  bool stalled(const SupervisorProgressPose & pose, int64_t now_ms);
  bool initialized() const;
  int64_t lastProgressMs() const;

private:
  SupervisorProgressConfig config_;
  bool initialized_{false};
  SupervisorProgressPose baseline_{};
  int64_t last_progress_ms_{0};
};
