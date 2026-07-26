#include "supervisor_progress.hpp"

#include <cassert>

int main()
{
  SupervisorProgress progress({0.05, 0.10, 10000});

  progress.reset({0.0, 0.0, 0.0}, 1000);

  // A detour that temporarily increases goal distance is still progress:
  // Nav2 checks displacement from the baseline pose, not goal-distance decrease.
  assert(!progress.stalled({0.06, 0.0, 0.0}, 9000));
  assert(!progress.stalled({0.06, 0.0, 0.0}, 18000));
  assert(progress.stalled({0.06, 0.0, 0.0}, 19001));

  // PoseProgressChecker semantics: meaningful rotation also refreshes progress.
  progress.reset({0.0, 0.0, 0.0}, 20000);
  assert(!progress.stalled({0.0, 0.0, 0.11}, 29000));
  assert(!progress.stalled({0.0, 0.0, 0.11}, 38000));
  assert(progress.stalled({0.0, 0.0, 0.11}, 39001));

  // Localization jitter below both thresholds must not hide a real stall.
  progress.reset({0.0, 0.0, 0.0}, 40000);
  assert(!progress.stalled({0.01, -0.01, 0.02}, 49999));
  assert(progress.stalled({0.01, -0.01, 0.02}, 50001));
  return 0;
}
