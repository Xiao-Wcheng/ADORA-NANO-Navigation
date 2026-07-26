#include "supervisor_goal.hpp"

#include <cmath>

namespace
{
double NormalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}
}

SupervisorGoalResult CheckSupervisorGoal(
  const SupervisorPose2D & pose, const SupervisorPose2D & goal,
  const SupervisorGoalTolerance & tolerance)
{
  SupervisorGoalResult result;
  result.position_error = std::hypot(goal.x - pose.x, goal.y - pose.y);
  result.yaw_error = std::abs(NormalizeAngle(goal.yaw - pose.yaw));
  result.position_reached = result.position_error <= tolerance.position;
  result.yaw_reached = result.yaw_error <= tolerance.yaw;
  result.reached = result.position_reached && result.yaw_reached;
  return result;
}

bool ShouldForwardLocalCommand(const std::string & nav_state, bool terminal_latched)
{
  return nav_state == "RUNNING" && !terminal_latched;
}
