# Narrow-Passage Navigation Implementation Plan

> **For agentic workers:** Execute inline using test-driven development. Do not
> create a Git commit.

**Goal:** Improve narrow-passage obstacle navigation and stop deterministic
micro-motion stalls.

**Architecture:** Production Dora templates extend MPPI look-ahead and reduce
dynamic inflation conservatively. A small testable supervisor progress
watchdog detects lack of net goal-distance improvement independently of the
local planner's mode string.

**Tech Stack:** C++17, CMake/CTest, Dora YAML.

## Global Constraints

- Keep `ROBOT_RADIUS=0.17` and `SAFETY_MARGIN=0.04`.
- Do not start the robot.
- Do not create a Git commit.

---

### Task 1: Add goal-progress watchdog

**Files:**
- Create: `planning/nav_supervisor_dora_node/src/supervisor_progress.hpp`
- Create: `planning/nav_supervisor_dora_node/src/supervisor_progress.cpp`
- Create: `planning/nav_supervisor_dora_node/tests/nav_supervisor_progress_test.cpp`
- Modify: `planning/nav_supervisor_dora_node/CMakeLists.txt`
- Modify: `planning/nav_supervisor_dora_node/src/nav_supervisor_dora_node.cpp`

- [ ] Add a failing unit test for progress, accumulated jitter, timeout, and reset.
- [ ] Run CMake/build and confirm RED due to the missing progress implementation.
- [ ] Implement the minimum watchdog and integrate it before command forwarding.
- [ ] Run CTest and confirm all supervisor tests pass.

### Task 2: Tune production obstacle parameters

**Files:**
- Modify: `apps/adora_nano_navigation/adora_nano_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_localization_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_slam_navigation.yml`

- [ ] Change all production `DYNAMIC_OBSTACLE_INFLATION_M` values to `0.21`.
- [ ] Change all production `MPPI_TIME_STEPS` values to `40`.
- [ ] Add `PROGRESS_TIMEOUT_MS=10000` and `MIN_GOAL_PROGRESS_M=0.02` to each
      production navigation supervisor.
- [ ] Generate a dry-run flow and verify the effective values.
- [ ] Run the full project build and all relevant CTests.
- [ ] Confirm Dora has no running flow.
