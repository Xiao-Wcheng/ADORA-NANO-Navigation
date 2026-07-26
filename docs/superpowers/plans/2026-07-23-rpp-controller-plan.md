# Pure Dora Nav2 RPP Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the production MPPI adapter with a deterministic ROS-free Nav2 Regulated Pure Pursuit controller for differential-style `vx + wz` navigation.

**Architecture:** A dedicated `rpp_local_controller_dora_node` owns Nav2-style path pruning, lookahead interpolation, regulated pure-pursuit velocity generation, and projected-arc collision checking. The existing Rotation Shim, goal checker, rolling costmap, Dora interfaces, velocity smoother, and supervisor are reused through copied focused support modules so the new node has no dependency on the retired MPPI implementation.

**Tech Stack:** C++17, Dora C node API, nlohmann JSON, Nav2 RPP Apache-2.0 algorithm port, CMake/CTest.

## Global Constraints

- Dora is the only runtime framework; no ROS 2 headers, libraries, messages, TF, pluginlib, or Costmap2D.
- Production motion uses `vx` and `wz`; every controller output has `vy == 0`.
- Robot radius remains `0.17 m` and safety margin remains `0.04 m`.
- Position tolerance remains `0.05 m` and yaw tolerance remains `0.035 rad`.
- Keep upstream Nav2 attribution and Apache-2.0 license text.
- Do not create a Git commit.
- Do not start hardware or a Dora navigation flow during implementation.

---

### Task 1: Nav2-Style Path Handler

**Files:**
- Create: `planning/rpp_local_controller_dora_node/include/nav2_rpp_port/types.hpp`
- Create: `planning/rpp_local_controller_dora_node/include/nav2_rpp_port/path_handler.hpp`
- Create: `planning/rpp_local_controller_dora_node/src/path_handler.cpp`
- Create: `planning/rpp_local_controller_dora_node/tests/path_handler_test.cpp`
- Create: `planning/rpp_local_controller_dora_node/CMakeLists.txt`

**Interfaces:**
- Consumes: map-frame `Path2D`, current `Pose2D`, `PathHandlerConfig{max_search_distance, max_local_path_length, minimum_pose_separation}`.
- Produces: `PathHandlerResult PathHandler::transformAndPrune(const Path2D &, const Pose2D &)`, containing `valid`, `reason`, `closest_index`, `pruned_count`, and robot-frame `local_path`.

- [ ] **Step 1: Write the failing path-handler test**

Assert that a robot at `(1.1, 0, 0)` on path points `(0,0)..(3,0)` prunes points behind it, returns a first local pose near the robot, preserves increasing forward geometry, and transforms correctly for robot yaw `pi/2`. Assert empty and non-finite paths are rejected.

- [ ] **Step 2: Configure and verify RED**

Run:

```bash
cmake -S planning/rpp_local_controller_dora_node -B planning/rpp_local_controller_dora_node/build
cmake --build planning/rpp_local_controller_dora_node/build --target path_handler_test -j2
```

Expected: compilation fails because `nav2_rpp_port/path_handler.hpp` or its implementation is absent.

- [ ] **Step 3: Implement bounded closest-pose selection and transformation**

Walk the global path only until integrated distance exceeds `max_search_distance`; choose the minimum Euclidean-distance pose in that window; begin the output at that index; transform each retained map pose using:

```cpp
const double dx = world.x - robot.x;
const double dy = world.y - robot.y;
body.x = std::cos(robot.yaw) * dx + std::sin(robot.yaw) * dy;
body.y = -std::sin(robot.yaw) * dx + std::cos(robot.yaw) * dy;
body.yaw = normalizeAngle(world.yaw - robot.yaw);
```

Stop after integrated local distance reaches `max_local_path_length`, skipping duplicate poses closer than `minimum_pose_separation`.

- [ ] **Step 4: Verify GREEN**

Run:

```bash
ctest --test-dir planning/rpp_local_controller_dora_node/build -R path_handler --output-on-failure
```

Expected: `1/1` passed.

### Task 2: Regulated Pure Pursuit Core and Collision Projection

**Files:**
- Create: `planning/rpp_local_controller_dora_node/include/nav2_rpp_port/rpp_controller.hpp`
- Create: `planning/rpp_local_controller_dora_node/src/rpp_controller.cpp`
- Create: `planning/rpp_local_controller_dora_node/include/nav2_rpp_port/costmap.hpp`
- Create: `planning/rpp_local_controller_dora_node/src/costmap.cpp`
- Create: `planning/rpp_local_controller_dora_node/tests/rpp_controller_test.cpp`
- Create: `planning/rpp_local_controller_dora_node/tests/costmap_test.cpp`
- Modify: `planning/rpp_local_controller_dora_node/CMakeLists.txt`

**Interfaces:**
- Consumes: robot-frame `Path2D`, current `Twist2D`, `RollingCostmap`, and `RppConfig`.
- Produces: `RppResult RppController::compute(const Path2D &, const Twist2D &, const RollingCostmap &)`, containing `valid`, `reason`, `command`, `lookahead_distance`, `carrot`, `curvature`, `regulation_factor`, and `collision_free`.

- [ ] **Step 1: Write failing controller tests**

Cover interpolated carrot selection, zero curvature for a straight path, positive/negative angular direction on left/right curves, strict `vy == 0`, curvature speed reduction, approach speed reduction, angular clamp, and zero/invalid result when the projected arc collides.

- [ ] **Step 2: Verify RED**

Run:

```bash
cmake --build planning/rpp_local_controller_dora_node/build --target rpp_controller_test -j2
```

Expected: compilation fails because `RppController` is not implemented.

- [ ] **Step 3: Port the minimal upstream RPP equations**

Use velocity-scaled lookahead:

```cpp
lookahead = std::clamp(
  std::abs(current.vx) * config.lookahead_time,
  config.min_lookahead, config.max_lookahead);
curvature = 2.0 * carrot.y /
  (carrot.x * carrot.x + carrot.y * carrot.y);
command.vx = regulated_linear_velocity;
command.vy = 0.0;
command.wz = std::clamp(
  command.vx * curvature, -config.max_angular_speed,
  config.max_angular_speed);
```

Apply Nav2 curvature regulation when `abs(1/curvature)` is below the configured minimum radius, clearance regulation from `RollingCostmap::clearanceAt(0,0)`, and approach scaling from remaining integrated path length. Prefer forward motion; paths with a carrot behind the robot return `rotate_to_path_required` instead of selecting reverse.

- [ ] **Step 4: Implement projected-arc collision checking**

Integrate `(vx, wz)` at `0.05 s` steps up to the smaller of collision horizon and carrot travel time. Check every pose with exact continuous obstacle distance using the unchanged `robot_radius + safety_margin`.

- [ ] **Step 5: Verify focused GREEN**

Run:

```bash
ctest --test-dir planning/rpp_local_controller_dora_node/build --output-on-failure
```

Expected: all RPP and costmap tests pass.

### Task 3: Dora Adapter, Rotation Shim, and Failure Semantics

**Files:**
- Create: `planning/rpp_local_controller_dora_node/include/nav2_rpp_port/rotation_shim.hpp`
- Create: `planning/rpp_local_controller_dora_node/src/rotation_shim.cpp`
- Create: `planning/rpp_local_controller_dora_node/include/nav2_rpp_port/goal_checker.hpp`
- Create: `planning/rpp_local_controller_dora_node/src/goal_checker.cpp`
- Create: `planning/rpp_local_controller_dora_node/include/nav2_rpp_port/adapter_state.hpp`
- Create: `planning/rpp_local_controller_dora_node/src/adapter_state.cpp`
- Create: `planning/rpp_local_controller_dora_node/src/rpp_local_controller_dora_node.cpp`
- Create: `planning/rpp_local_controller_dora_node/tests/adapter_state_test.cpp`
- Create: `planning/rpp_local_controller_dora_node/tests/rotation_shim_test.cpp`
- Modify: `planning/rpp_local_controller_dora_node/CMakeLists.txt`
- Modify: `planning/nav_supervisor_dora_node/src/nav_supervisor_dora_node.cpp`
- Modify: `planning/nav_supervisor_dora_node/tests/nav_supervisor_progress_test.cpp`

**Interfaces:**
- Consumes Dora inputs `CorrectedPose`, `path`, `GoalPose`, `LaserScan`, and `tick`.
- Produces unchanged outputs `CmdVelTwist` and `LocalPlannerStatus`, with `controller_impl=nav2_regulated_pure_pursuit_port`.

- [ ] **Step 1: Write failing failure-state tests**

Assert `path_found=false` maps immediately to `BLOCKED/no_global_path`; `ROTATE_TO_PATH` carries only angular velocity; `FOLLOW_PATH` has `vy=0`; and the supervisor maps local `BLOCKED/no_global_path` to supervisor `BLOCKED` without waiting for the progress timeout.

- [ ] **Step 2: Verify RED**

Run the new adapter target and existing supervisor tests. Expected: assertions fail because no-global-path is currently treated as an empty stale path and supervisor remains `RUNNING`.

- [ ] **Step 3: Integrate the fixed 100 ms control sequence**

On each `tick`:

1. Validate input freshness and localization.
2. If the latest global planner message says `path_found=false`, publish zero and `BLOCKED/no_global_path`.
3. Transform and prune the current global path.
4. Refresh the rolling laser costmap.
5. If goal position is reached, run final-yaw alignment.
6. Otherwise run Rotation Shim.
7. If shim is inactive, run RPP.
8. Pass the result through adapter safety checks and publish status/command.

Reset path-handler state, shim state, and controller command history on every new goal.

- [ ] **Step 4: Add status diagnostics**

Publish local/pruned path counts, carrot, lookahead, curvature, regulation factor, collision result, localization health, and exact failure reason.

- [ ] **Step 5: Verify component tests**

Run RPP and supervisor CTest suites. Expected: zero failures.

### Task 4: Production Wiring, Attribution, and Full Verification

**Files:**
- Create: `planning/rpp_local_controller_dora_node/UPSTREAM.md`
- Create: `planning/rpp_local_controller_dora_node/LICENSE.nav2-rpp-apache-2.0`
- Modify: `scripts/build_all.sh`
- Modify: `apps/adora_nano_navigation/adora_nano_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_localization_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_slam_navigation.yml`
- Modify: `apps/adora_nano_navigation/tests/test_mppi_production_wiring.py`

**Interfaces:**
- Replaces the production executable path with `../../planning/rpp_local_controller_dora_node/build/rpp_local_controller_dora_node`.
- Adds identical `RPP_*` configuration defaults to all three navigation dataflows.

- [ ] **Step 1: Add production defaults**

Use conservative Adora Nano values:

```yaml
RPP_DESIRED_LINEAR_VEL: '0.04'
RPP_MIN_LOOKAHEAD_DIST: '0.20'
RPP_MAX_LOOKAHEAD_DIST: '0.45'
RPP_LOOKAHEAD_TIME: '1.5'
RPP_REGULATED_MIN_RADIUS: '0.40'
RPP_REGULATED_MIN_SPEED: '0.012'
RPP_APPROACH_SCALING_DIST: '0.35'
RPP_MIN_APPROACH_SPEED: '0.008'
RPP_MAX_ANGULAR_SPEED: '0.20'
RPP_COLLISION_HORIZON: '1.5'
RPP_COLLISION_DT: '0.05'
```

Retain the existing Rotation Shim, costmap, laser extrinsics, tolerances, and timeout values.

- [ ] **Step 2: Record upstream provenance**

Document the exact Navigation2 repository URL, package path, Apache-2.0 license, upstream equations ported, ROS-dependent pieces deliberately replaced, and retrieval date.

- [ ] **Step 3: Build and test the complete standalone project**

Run:

```bash
./scripts/build_all.sh
ctest --test-dir planning/rpp_local_controller_dora_node/build --output-on-failure
ctest --test-dir localization/karto_slam_dora_node/build --output-on-failure
ctest --test-dir planning/adora_nano_global_planner_node/build --output-on-failure
ctest --test-dir planning/nav_supervisor_dora_node/build --output-on-failure
ctest --test-dir planning/velocity_smoother_dora_node/build --output-on-failure
python3 -m unittest discover -s apps/adora_nano_navigation/tests
```

Expected: every build and test exits `0`.

- [ ] **Step 4: Verify standalone readiness**

Run:

```bash
./scripts/check_ready.sh
./apps/adora_nano_navigation/verify_ros_free.sh
```

Expected: `Result: READY` and `ROS_FREE=PASS`.

- [ ] **Step 5: Confirm hardware remains stopped**

Check exact process paths for lidar, chassis, localization, global planner, RPP, smoother, and supervisor. Expected: no live matching process.
