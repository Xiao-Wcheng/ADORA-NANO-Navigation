#include "supervisor_goal.hpp"

#include <cassert>
#include <cmath>

int main()
{
  const SupervisorGoalTolerance tolerance{0.08, 0.12};
  auto result = CheckSupervisorGoal({0.01, 0.01, 0.50}, {0, 0, 0}, tolerance);
  assert(!result.reached && result.position_reached && !result.yaw_reached);
  result = CheckSupervisorGoal({0.20, 0.0, 0.01}, {0, 0, 0}, tolerance);
  assert(!result.reached && !result.position_reached && result.yaw_reached);
  result = CheckSupervisorGoal({0.03, 0.02, 0.10}, {0, 0, 0}, tolerance);
  assert(result.reached);
  result = CheckSupervisorGoal(
    {0, 0, -M_PI + 0.04}, {0, 0, M_PI - 0.04}, tolerance);
  assert(result.reached && std::abs(result.yaw_error - 0.08) < 1e-12);

  assert(ShouldForwardLocalCommand("RUNNING", false));
  assert(!ShouldForwardLocalCommand("REACHED", true));
  assert(!ShouldForwardLocalCommand("TIMEOUT", true));
  assert(!ShouldForwardLocalCommand("LOCALIZATION_LOST", true));
  return 0;
}
