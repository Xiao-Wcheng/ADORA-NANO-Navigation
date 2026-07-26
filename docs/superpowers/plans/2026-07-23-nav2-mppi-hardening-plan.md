# Nav2 MPPI Dora Adapter Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port mature Nav2 MPPI retry/configuration and Velocity Smoother deadband behavior into the pure-Dora navigation stack.

**Architecture:** Keep MPPI responsible for trajectory optimization, add Nav2-style bounded soft retries, and place deadband/acceleration handling in a separate velocity-smoothing unit before supervisor forwarding. Preserve the independent safety-stop layer.

**Tech Stack:** C++17, Dora C node API, CMake/CTest, nlohmann/json.

## Global Constraints

- No ROS 2 runtime dependency.
- Keep `SAFETY_MARGIN=0.04`.
- Zero, stale, lost-localization, and blocked inputs must remain zero.
- Do not start real navigation during software verification.
- Do not commit Git changes.

---

### Task 1: Critic environment configuration

**Files:**
- Modify: `planning/mppi_local_planner_dora_node/src/mppi_local_planner_dora_node.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/critics_test.cpp`

**Interfaces:**
- Produces: `CriticConfig criticConfig(const Config &)` passed into `OptimizerConfig::critics`.

- [ ] Add a failing test showing changed path, goal, obstacle, heading, effort, smoothness, and deadband weights change the corresponding score.
- [ ] Run `cmake --build planning/mppi_local_planner_dora_node/build && ctest --test-dir planning/mppi_local_planner_dora_node/build -R critics_test --output-on-failure`; expect the new configuration test to fail.
- [ ] Load each existing YAML environment variable into `Config`, construct `CriticConfig`, and assign it in `optimizerConfig`.
- [ ] Run the same test; expect it to pass.

### Task 2: Nav2-style bounded soft retry

**Files:**
- Modify: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/optimizer.hpp`
- Modify: `planning/mppi_local_planner_dora_node/src/optimizer.cpp`
- Modify: `planning/mppi_local_planner_dora_node/src/mppi_local_planner_dora_node.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/optimizer_test.cpp`

**Interfaces:**
- Consumes: `OptimizerConfig::retry_attempt_limit`.
- Produces: `OptimizerResult::retry_count`, `finite_trajectory_count`, and `collision_trajectory_count`.

- [ ] Add a failing deterministic test for bounded retry and diagnostic counters.
- [ ] Run the optimizer test; expect failure because retry fields and behavior do not exist.
- [ ] Port the Nav2 soft-failure loop: reset the nominal sequence/noise and retry up to `retry_attempt_limit`; return failure only after the limit.
- [ ] Count finite and collision-rejected trajectories without changing collision thresholds.
- [ ] Run optimizer tests; expect all to pass.

### Task 3: Nav2 Velocity Smoother port

**Files:**
- Create: `planning/velocity_smoother_dora_node/src/velocity_smoother.cpp`
- Create: `planning/velocity_smoother_dora_node/include/nav2_velocity_smoother_port/velocity_smoother.hpp`
- Create: `planning/velocity_smoother_dora_node/tests/velocity_smoother_test.cpp`
- Create: `planning/velocity_smoother_dora_node/CMakeLists.txt`
- Modify: `apps/adora_nano_navigation/adora_nano_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_localization_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_slam_navigation.yml`

**Interfaces:**
- Consumes: MPPI `CmdVelTwist`.
- Produces: `SmoothedCmdVelTwist` for the supervisor.
- Parameters: `SMOOTHING_FREQUENCY`, `DEADBAND_VX`, `DEADBAND_VY`, `DEADBAND_WZ`, axis velocity/acceleration/deceleration limits, `VELOCITY_TIMEOUT`.

- [ ] Write failing unit tests for zero preservation, deadband suppression, acceleration limiting, deceleration limiting, and timeout stop.
- [ ] Build and run the new test; expect failure before implementation.
- [ ] Port the relevant open-loop Nav2 Velocity Smoother equations into the ROS-independent library.
- [ ] Add the Dora adapter node and route MPPI output through it to supervisor.
- [ ] Run the new unit tests; expect all to pass.

### Task 4: Configuration generation and verification

**Files:**
- Modify: `apps/adora_nano_navigation/adora_nav.py`
- Modify: `scripts/check_ready.sh`
- Test: existing generated navigation dry-run and all CTest suites.

**Interfaces:**
- Produces: generated navigation YAML containing the same smoother and MPPI retry parameters as all production templates.

- [ ] Add a failing configuration-generation assertion for the smoother node, retry limit, and critic weights.
- [ ] Update generation/validation code and rebuild all native nodes.
- [ ] Run all MPPI, smoother, supervisor, and global planner tests.
- [ ] Run `./scripts/build_all.sh` and `./scripts/check_ready.sh`.
- [ ] Inspect the final diff and confirm no ROS runtime linkage and no safety parameter reduction.
