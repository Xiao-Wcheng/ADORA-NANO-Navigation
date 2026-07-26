# Pure Dora Rotation Shim Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Nav2-style rotation shim that aligns the circular omni robot with the initial global-path segment before MPPI translation.

**Architecture:** A small stateful `RotationShim` module consumes the body-frame path and current-footprint clearance, applies enter/exit hysteresis, and returns a zero-linear-velocity angular command. The Dora adapter invokes it on the existing fixed 100 ms control tick before MPPI and exposes a distinct `ROTATE_TO_PATH` status.

**Tech Stack:** C++17, Dora C node API, nlohmann JSON, existing `nav2_mppi_port` types and continuous-distance rolling costmap.

## Global Constraints

- Pure Dora implementation; no ROS runtime or ROS messages.
- Defaults: enter at `0.75 rad`, exit at `0.35 rad`, maximum angular speed `0.20 rad/s`.
- Rotation output always has `vx=0` and `vy=0`.
- Existing localization, map, A*, robot radius, and `0.04 m` safety margin remain unchanged.
- Do not create a Git commit.

---

### Task 1: Rotation Shim State Machine

**Files:**
- Create: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/rotation_shim.hpp`
- Create: `planning/mppi_local_planner_dora_node/src/rotation_shim.cpp`
- Create: `planning/mppi_local_planner_dora_node/tests/rotation_shim_test.cpp`
- Modify: `planning/mppi_local_planner_dora_node/CMakeLists.txt`

**Interfaces:**
- Consumes: `Path2D`, `Twist2D`, and `normalizeAngle` from `nav2_mppi_port/types.hpp`.
- Produces: `RotationShimConfig`, `RotationShimResult`, and `RotationShim::compute(const Path2D &, bool footprint_clear)`.

- [ ] **Step 1: Write the failing state-machine test**

The test must assert entry above `0.75 rad`, zero linear command, shortest-turn direction across `-pi/pi`, persistence in the hysteresis band, exit below `0.35 rad`, angular clamp, and a blocked zero command when `footprint_clear=false`.

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
cmake --build planning/mppi_local_planner_dora_node/build --target rotation_shim_test -j2
```

Expected: compilation fails because `rotation_shim.hpp` does not exist.

- [ ] **Step 3: Implement the minimal state machine**

Use the first non-degenerate path segment. Compute its heading with `atan2`, normalize the error, use separate enter/exit thresholds, proportional angular control with minimum useful angular speed and maximum clamp, and return zero translation in every active result.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run:

```bash
ctest --test-dir planning/mppi_local_planner_dora_node/build -R rotation_shim --output-on-failure
```

Expected: `1/1` passed.

### Task 2: Dora Adapter Integration

**Files:**
- Modify: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/adapter_state.hpp`
- Modify: `planning/mppi_local_planner_dora_node/src/adapter_state.cpp`
- Modify: `planning/mppi_local_planner_dora_node/src/mppi_local_planner_dora_node.cpp`
- Modify: `planning/mppi_local_planner_dora_node/tests/adapter_state_test.cpp`
- Modify: `planning/mppi_local_planner_dora_node/CMakeLists.txt`

**Interfaces:**
- Consumes: `RotationShim::compute`, `RollingCostmap::trajectoryCollisionFree`, fixed `tick` control gate, current goal-check result, and body-frame path.
- Produces: `LocalMode::ROTATE_TO_PATH`, status reason `rotate_to_path`, and environment configuration keys `ROTATION_SHIM_*`.

- [ ] **Step 1: Write failing adapter-state assertions**

Add assertions that `toString(LocalMode::ROTATE_TO_PATH)` returns `ROTATE_TO_PATH` and that the mode carries a finite zero-linear angular command.

- [ ] **Step 2: Run the adapter test and verify RED**

Run:

```bash
cmake --build planning/mppi_local_planner_dora_node/build --target adapter_state_test -j2
```

Expected: compilation fails because `ROTATE_TO_PATH` is not defined.

- [ ] **Step 3: Integrate the shim on fixed control ticks**

Load defaults and environment overrides for enter threshold, exit threshold, gain, minimum angular speed, and maximum angular speed. After refreshing the costmap, evaluate shim only when position is not reached. If active and clear, bypass MPPI for that tick and publish its angular command with `ROTATE_TO_PATH`; if active and not clear, publish zero with `BLOCKED`; otherwise call MPPI unchanged. Reset shim on a new goal.

- [ ] **Step 4: Run MPPI component tests**

Run:

```bash
ctest --test-dir planning/mppi_local_planner_dora_node/build --output-on-failure
```

Expected: all MPPI tests pass.

### Task 3: Configuration and Full Verification

**Files:**
- Modify: `apps/adora_nano_navigation/adora_nano_localization_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_slam_navigation.yml`

**Interfaces:**
- Adds identical `ROTATION_SHIM_*` defaults to each local-planner environment.
- Leaves generated runtime YAML to `run_navigation.py`.

- [ ] **Step 1: Add explicit dataflow defaults**

Set:

```yaml
ROTATION_SHIM_ENTER_ANGLE: '0.75'
ROTATION_SHIM_EXIT_ANGLE: '0.35'
ROTATION_SHIM_ANGULAR_GAIN: '1.0'
ROTATION_SHIM_MIN_ANGULAR_SPEED: '0.05'
ROTATION_SHIM_MAX_ANGULAR_SPEED: '0.20'
```

- [ ] **Step 2: Build the complete project**

Run:

```bash
./scripts/build_all.sh
```

Expected: exit code `0`.

- [ ] **Step 3: Run all navigation-stack tests**

Run CTest for MPPI, Karto, global planner, supervisor, and velocity smoother.

Expected: zero failed tests.

- [ ] **Step 4: Verify standalone readiness**

Run:

```bash
./scripts/check_ready.sh
./apps/adora_nano_navigation/verify_ros_free.sh
```

Expected: `Result: READY` and `ROS_FREE=PASS`.

- [ ] **Step 5: Confirm no live robot processes**

Use exact-name `pgrep` checks for lidar, chassis, localization, global planner, MPPI, smoother, and supervisor.

Expected: no matching processes.
