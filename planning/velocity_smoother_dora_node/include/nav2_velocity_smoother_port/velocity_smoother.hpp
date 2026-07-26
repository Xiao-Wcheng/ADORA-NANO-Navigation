#pragma once

#include <array>
#include <cstdint>

namespace nav2_velocity_smoother_port
{

struct Twist
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct Config
{
  double frequency{20.0};
  std::array<double, 3> max_velocity{{0.045, 0.045, 0.20}};
  std::array<double, 3> min_velocity{{-0.045, -0.045, -0.20}};
  std::array<double, 3> max_accel{{0.10, 0.10, 0.50}};
  std::array<double, 3> max_decel{{-0.10, -0.10, -0.50}};
  std::array<double, 3> deadband{{0.005, 0.005, 0.02}};
  int64_t velocity_timeout_ms{350};
};

class VelocitySmoother
{
public:
  explicit VelocitySmoother(Config config);
  void setTarget(const Twist & target, int64_t now_ms);
  Twist update(int64_t now_ms);
  void reset();

private:
  Config config_;
  Twist target_{};
  Twist current_{};
  int64_t last_target_ms_{-1};
};

}  // namespace nav2_velocity_smoother_port
