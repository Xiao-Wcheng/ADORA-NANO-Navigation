# Pure-Dora MPPI Omni Local Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the production DWB controller with a ROS-free Navigation2 MPPI Omni controller that commands `vx`, `vy`, and `wz` concurrently and reaches both position and yaw tolerances.

**Architecture:** Preserve Navigation2 MPPI's control-sequence sampling, Omni forward model, critic batch scoring, softmax update, sequence shift, and collision validation behind ROS-free C++17 interfaces. A thin Dora adapter supplies Karto pose, A* path, MS200 scans, and goal pose, while the existing navigation supervisor remains the only final chassis command gate.

**Tech Stack:** C++17, Dora C API, nlohmann JSON, CMake/CTest, Python 3 structural tests, Navigation2 MPPI upstream commit `db906947171abe170c25181347be9bc7bcbc1a75`.

## Global Constraints

- No ROS installation or runtime, ROS messages, TF, pluginlib, lifecycle nodes, ament, or ROS workspace.
- No Git commits.
- Position tolerance is exactly `0.08 m`.
- Wrapped yaw tolerance is exactly `0.12 rad`.
- Initial velocity limits are `vx/vy = -0.045..0.045 m/s` and `wz = -0.20..0.20 rad/s`.
- Initial acceleration limits are `0.10 m/s^2` per linear axis and `0.50 rad/s^2` angular.
- Navigation timeout is `180000 ms`.
- DWB is deleted only after MPPI automated, wiring, bench, and physical verification pass.

---

### Task 1: Upstream Provenance and ROS-Free Skeleton

**Files:**
- Create: `planning/mppi_local_planner_dora_node/CMakeLists.txt`
- Create: `planning/mppi_local_planner_dora_node/LICENSE.nav2-apache-2.0`
- Create: `planning/mppi_local_planner_dora_node/UPSTREAM.md`
- Create: `planning/mppi_local_planner_dora_node/MODIFICATIONS.md`
- Create: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/types.hpp`
- Test: `planning/mppi_local_planner_dora_node/tests/mppi_types_test.cpp`

**Interfaces:**
- Produces: `Pose2D`, `Twist2D`, `Path2D`, `ControlSequence`, `TrajectoryBatch`, `ObstaclePoint`, `GoalTolerance`, and `normalizeAngle(double)` in namespace `nav2_mppi_port`.

- [ ] Copy the exact Apache-2.0 license and record upstream commit `db906947171abe170c25181347be9bc7bcbc1a75` plus the used `nav2_mppi_controller` source paths.
- [ ] Write a failing test asserting angle wrapping and fixed-size sequence/batch shape invariants.
- [ ] Configure only the test target, run CMake/CTest, and verify RED because the types do not exist.
- [ ] Add the minimal POD types and angle helper, then verify the test passes.
- [ ] Run `ldd` on the test and confirm no ROS library appears.

### Task 2: Omni Motion Model and Constraints

**Files:**
- Create: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/omni_motion_model.hpp`
- Create: `planning/mppi_local_planner_dora_node/src/omni_motion_model.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/omni_motion_model_test.cpp`

**Interfaces:**
- Consumes: `Pose2D`, `Twist2D`, and `ControlSequence`.
- Produces: `OmniConstraints::clamp`, `OmniMotionModel::predict`, and `OmniMotionModel::predictBatch`.

- [ ] Write a failing test whose trajectory changes `x`, `y`, and `yaw` from simultaneous non-zero `vx/vy/wz`.
- [ ] Add failing cases for velocity and per-step acceleration limits.
- [ ] Verify RED against the skeleton.
- [ ] Port the upstream Omni model equations and constraint behavior without ROS types.
- [ ] Verify all model tests pass, including negative lateral and wrapped yaw cases.

### Task 3: Noise Sampling and Control Sequence Management

**Files:**
- Create: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/noise_generator.hpp`
- Create: `planning/mppi_local_planner_dora_node/src/noise_generator.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/noise_generator_test.cpp`

**Interfaces:**
- Produces: `NoiseConfig`, `NoiseGenerator::sample`, `applyNoise`, and `shiftControlSequence`.

- [ ] Write failing tests for deterministic seeded `vx/vy/wz` noise, finite values, configured standard deviations, and one-step sequence shifting.
- [ ] Verify RED.
- [ ] Implement the minimal standard-library Gaussian generator and upstream sequence-shift semantics.
- [ ] Verify deterministic and statistical-bound tests pass.

### Task 4: Rolling Lidar Costmap and Collision Checking

**Files:**
- Create: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/costmap.hpp`
- Create: `planning/mppi_local_planner_dora_node/src/costmap.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/costmap_test.cpp`

**Interfaces:**
- Produces: `RollingCostmap::reset`, `insertScanPoints`, `inflate`, `costAt`, `clearanceAt`, and `trajectoryCollisionFree`.

- [ ] Write a failing test that inserts a synthetic obstacle, inflates it by robot radius plus safety margin, and rejects a crossing trajectory while accepting a clear trajectory.
- [ ] Add failing tests for bounds, non-finite points, and the calibrated lidar-to-base transform.
- [ ] Verify RED.
- [ ] Implement robot-centered rasterization, Euclidean inflation, clearance lookup, and swept trajectory collision checks.
- [ ] Verify all costmap tests pass.

### Task 5: MPPI Critics

**Files:**
- Create: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/critics.hpp`
- Create: `planning/mppi_local_planner_dora_node/src/critics.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/critics_test.cpp`

**Interfaces:**
- Produces: `CriticConfig` and `CriticManager::scoreBatch` with path-follow, path-align, goal-distance, goal-angle, obstacle, deadband, effort, and smoothness terms.

- [ ] Write one failing preference test per critic using two otherwise identical trajectories.
- [ ] Verify RED and confirm each assertion fails for the missing intended preference.
- [ ] Port the upstream cost equations and static critic composition.
- [ ] Verify individual tests and a combined finite-cost test pass.

### Task 6: MPPI Optimizer

**Files:**
- Create: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/optimizer.hpp`
- Create: `planning/mppi_local_planner_dora_node/src/optimizer.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/optimizer_test.cpp`

**Interfaces:**
- Produces: `OptimizerConfig`, `OptimizerInput`, `OptimizerResult`, `MppiOptimizer::compute`, and `MppiOptimizer::reset`.

- [ ] Write a failing clear-space test requiring simultaneous non-zero translation and rotation toward a pose goal.
- [ ] Write failing tests for softmax numerical stability, collision rejection, repeatable seeded output, sequence reuse, and no-valid-trajectory zero result.
- [ ] Verify RED.
- [ ] Implement batched prediction, critic scoring, minimum-cost normalization, exponential weighting, control update, constraint clamp, shift, retry, and final collision validation.
- [ ] Benchmark the test host and select a batch/horizon that completes below `80 ms` at the 95th percentile, leaving margin for the `100 ms` control tick.
- [ ] Verify all optimizer tests pass and record the benchmark parameters.

### Task 7: Combined Position and Yaw Goal Checking

**Files:**
- Create: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/goal_checker.hpp`
- Create: `planning/mppi_local_planner_dora_node/src/goal_checker.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/goal_checker_test.cpp`

**Interfaces:**
- Produces: `GoalCheckResult checkGoal(current, goal, position_tolerance, yaw_tolerance)` with position error, wrapped yaw error, and `reached`.

- [ ] Write failing tests for position-only success being rejected, yaw-only success being rejected, joint success, and `+pi/-pi` wrapping.
- [ ] Verify RED.
- [ ] Implement the minimal combined checker with `0.08 m` and `0.12 rad` defaults.
- [ ] Verify all goal tests pass.

### Task 8: Dora Adapter and Fail-Safe State Machine

**Files:**
- Create: `planning/mppi_local_planner_dora_node/src/mppi_local_planner_dora_node.cpp`
- Test: `planning/mppi_local_planner_dora_node/tests/adapter_state_test.cpp`
- Modify: `planning/mppi_local_planner_dora_node/CMakeLists.txt`

**Interfaces:**
- Consumes: existing `path`, `LaserScan`, `CorrectedPose`, `GoalPose`, and timer JSON contracts.
- Produces: existing `CmdVelTwist` and `LocalPlannerStatus` contracts with modes `WAITING`, `FOLLOW_PATH`, `ALIGN_GOAL`, `BLOCKED`, `LOCALIZATION_LOST`, and `GOAL_REACHED`.

- [ ] Extract the existing proven JSON parsing and timestamp conventions into adapter-only helpers.
- [ ] Write failing state tests for stale pose/path/scan, localization loss, malformed/non-finite input, blocked trajectory, follow path, align goal, and joint goal reached.
- [ ] Verify RED.
- [ ] Implement the adapter state machine so every non-running state emits explicit zero velocity.
- [ ] Link the Dora executable only after algorithm/state tests are green.
- [ ] Verify unit tests, Dora node build, and no-ROS `ldd` scan.

### Task 9: Supervisor Dual-Tolerance and Timeout Alignment

**Files:**
- Modify: `planning/nav_supervisor_dora_node/src/nav_supervisor_dora_node.cpp`
- Create or modify: `planning/nav_supervisor_dora_node/tests/nav_supervisor_test.cpp`
- Modify: `planning/nav_supervisor_dora_node/CMakeLists.txt`

**Interfaces:**
- Consumes: goal yaw and MPPI status yaw error.
- Produces: terminal `REACHED` only when position and yaw tolerances both pass; timeout remains latched and commands zero.

- [ ] Write failing supervisor tests proving position-only arrival is not `REACHED`, joint arrival is `REACHED`, and timeout zeroes non-zero MPPI input.
- [ ] Verify RED.
- [ ] Add `YAW_GOAL_TOLERANCE=0.12` and default `NAV_TIMEOUT_MS=180000` while preserving the safety gate.
- [ ] Verify supervisor and existing gate tests pass.

### Task 10: Production Dataflows and Launchers

**Files:**
- Modify: `apps/adora_nano_navigation/adora_nano_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_localization_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_slam_navigation.yml`
- Modify: `apps/adora_nano_navigation/run_navigation.py`
- Modify: `scripts/build_all.sh`
- Modify: `apps/adora_nano_navigation/build_navigation_stack.sh`
- Modify: `scripts/check_ready.sh`
- Modify: `tests/test_entrypoints.py`
- Modify: `tests/test_standalone_paths.py`

**Interfaces:**
- Replaces the production executable path while keeping Dora topic IDs stable.

- [ ] Write failing structural assertions requiring the MPPI executable/config and forbidding production DWB references.
- [ ] Verify RED.
- [ ] Replace the node path and MPPI environment block in all production templates; set velocity, acceleration, tolerances, lidar transform, costmap, batch/horizon, seed, and `180000 ms` timeout values.
- [ ] Update generated-flow logic and readiness/build scripts.
- [ ] Verify Python tests, YAML parsing, and Dora validation for localization, replan, and SLAM modes.

### Task 11: Documentation, License, and Manifest Migration

**Files:**
- Modify: `README.md`
- Modify: `docs/COMPONENTS.md`
- Modify: `docs/OPERATIONS.md`
- Modify: `docs/OPEN_SOURCE.md`
- Modify: `docs/VALIDATION.md`
- Modify: `apps/adora_nano_navigation/INTERFACES.md`
- Modify: `docs/standalone_navigation_manifest.txt`
- Modify: `LICENSES/README.md`

- [ ] Add failing manifest/documentation scans requiring MPPI provenance and forbidding DWB as a production component.
- [ ] Verify RED.
- [ ] Document exact upstream revision, adaptation boundary, tuning, simultaneous motion, dual tolerance, timeout, safety, and validation commands.
- [ ] Verify manifest, external-path, and license scans pass.

### Task 12: Automated and Bench Verification

**Files:**
- Modify: `docs/VALIDATION.md`

- [ ] Run a clean MPPI build and every compiled/Python test; require zero failures.
- [ ] Run Dora dry-runs and validation for mapping, localization navigation, replan, and SLAM navigation.
- [ ] Run `ldd` across production binaries and require no ROS libraries.
- [ ] Run source/path scans and require no old repository runtime access.
- [ ] With wheels lifted, command a short pose change and confirm observed commands include simultaneous non-zero `vx/vy/wz`, emergency stop produces zero, and the process exits cleanly.
- [ ] Record evidence in `docs/VALIDATION.md`.

### Task 13: Physical MPPI Acceptance and DWB Removal

**Files:**
- Delete after acceptance: `planning/dwb_local_planner_dora_node/`
- Modify: `docs/standalone_navigation_manifest.txt`
- Modify: `docs/VALIDATION.md`

- [ ] Run the recorded clear-floor goal `(-0.634, -0.185, 1.545)` and require decreasing position error, `loc_lost=0`, final position error at most `0.08 m`, final yaw error at most `0.12 rad`, `REACHED`, and latched zero velocity.
- [ ] Repeat with a lidar-visible temporary obstacle and require collision-free avoidance or a safe blocked stop.
- [ ] Stop all flows and confirm no navigation processes remain.
- [ ] Remove the DWB directory and all remaining DWB build/runtime/documentation references.
- [ ] Re-run the complete build, test, Dora validation, ROS-link, manifest, and process-cleanup suite.
- [ ] Record final physical results without creating a Git commit.
