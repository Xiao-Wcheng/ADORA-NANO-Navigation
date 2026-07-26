#pragma once

#include <string>

struct SupervisorPose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SupervisorGoalTolerance
{
  double position{0.08};
  double yaw{0.12};
};

struct SupervisorGoalResult
{
  double position_error{0.0};
  double yaw_error{0.0};
  bool position_reached{false};
  bool yaw_reached{false};
  bool reached{false};
};

SupervisorGoalResult CheckSupervisorGoal(
  const SupervisorPose2D & pose, const SupervisorPose2D & goal,
  const SupervisorGoalTolerance & tolerance);

bool ShouldForwardLocalCommand(const std::string & nav_state, bool terminal_latched);
