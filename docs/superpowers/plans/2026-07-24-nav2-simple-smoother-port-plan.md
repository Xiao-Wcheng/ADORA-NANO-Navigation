# Nav2 SimpleSmoother Standalone Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace custom endpoint-only LOS compression with an Apache-2.0 standalone port of Nav2 SimpleSmoother that preserves dense paths for RPP.

**Architecture:** Boost A* continues to produce integer grid poses. A standalone `Nav2SimpleSmoother` converts them to floating-point grid poses, runs the official Nav2 data/smooth weighted iteration with collision rollback and refinement, and publishes the same JSON schema with numeric `x/y`. The RPP adapter reads those coordinates as doubles and converts them to world poses without truncation.

**Tech Stack:** C++17, Boost Graph A*, yaml-cpp, nlohmann/json, CMake/CTest, Dora C API.

## Global Constraints

- No ROS runtime or ROS package dependency.
- Preserve the Nav2 SimpleSmoother formula and defaults: tolerance `1e-10`, max iterations `1000`, data weight `0.2`, smooth weight `0.3`, refinement enabled, refinement count `2`.
- Preserve Apache-2.0 attribution and upstream source URL.
- Keep the existing Dora JSON field names and safety behavior.
- Do not create a Git commit.

---

### Task 1: Standalone Nav2 SimpleSmoother core

**Files:**
- Create: `planning/adora_nano_global_planner_node/include/nav2_simple_smoother.h`
- Create: `planning/adora_nano_global_planner_node/src/nav2_simple_smoother.cpp`
- Create: `planning/adora_nano_global_planner_node/tests/nav2_simple_smoother_test.cpp`
- Modify: `planning/adora_nano_global_planner_node/CMakeLists.txt`
- Delete: `planning/adora_nano_global_planner_node/include/path_smoothing.h`
- Delete: `planning/adora_nano_global_planner_node/src/path_smoothing.cpp`
- Delete: `planning/adora_nano_global_planner_node/tests/path_smoothing_test.cpp`

**Interfaces:**
- Consumes: `std::vector<Point>` from Boost A* and `MapLoader::isValid/isOccupied`.
- Produces: `std::vector<GridPathPoint> Nav2SimpleSmoother::smooth(const std::vector<Point>&) const`.

- [ ] Add a failing test with a dense 52-point corner path that requires output size `52`, fixed endpoints, reduced second-difference roughness, and collision-free points.
- [ ] Configure and build the test; verify RED because `nav2_simple_smoother.h` does not exist.
- [ ] Port the official Nav2 update formula into a ROS-free class using `GridPathPoint {double x, double y}`, collision rollback, iteration limit, tolerance, and two refinements.
- [ ] Add the upstream copyright, Apache-2.0 notice, and source URL to both new files.
- [ ] Replace the old LOS source/test targets in CMake and delete the obsolete implementation.
- [ ] Build and run `ctest --test-dir planning/adora_nano_global_planner_node/build --output-on-failure`; expect all global-planner tests to pass.

### Task 2: Runtime and RPP floating-point path contract

**Files:**
- Modify: `planning/adora_nano_global_planner_node/src/dora_planning_node.cpp`
- Modify: `planning/rpp_local_controller_dora_node/src/rpp_local_controller_dora_node.cpp`
- Create: `planning/rpp_local_controller_dora_node/tests/path_json_contract_test.cpp`
- Modify: `planning/rpp_local_controller_dora_node/CMakeLists.txt`

**Interfaces:**
- Consumes: smoothed `std::vector<GridPathPoint>`.
- Produces: existing JSON `waypoints` array with floating-point `x/y`; RPP reads them as `double`.

- [ ] Add a failing RPP contract test showing waypoint `x=17.5` and `y=23.25` survive JSON-to-world conversion without integer truncation.
- [ ] Run the specific test and verify RED against the current integer parser.
- [ ] Extract or expose the JSON path conversion helper and change `gx/iy` from `int` to `double`.
- [ ] Replace runtime LOS smoothing with `Nav2SimpleSmoother`, publish dense floating-point poses, and report `path_smoother="nav2_simple_smoother_port"`.
- [ ] Build both nodes and run both CTest suites with zero failures.

### Task 3: Integration and stationary runtime verification

**Files:**
- Modify only if needed: `apps/adora_nano_navigation/adora_nav.py` generated environment wiring.
- Update: `docs/superpowers/specs/2026-07-24-open-source-dense-path-rpp-design.md`

- [ ] Run `./scripts/build_all.sh`.
- [ ] Run `./scripts/check_ready.sh` and validate the generated navigation dataflow.
- [ ] Run Python project tests.
- [ ] Start position1 navigation at the verified origin only long enough to inspect logs while physically preventing unintended movement via immediate stop readiness.
- [ ] Verify global output has more than two points, RPP has at least two local points, and `invalid_path` is absent.
- [ ] Stop all Dora flows and report that real motion acceptance remains for the user.
