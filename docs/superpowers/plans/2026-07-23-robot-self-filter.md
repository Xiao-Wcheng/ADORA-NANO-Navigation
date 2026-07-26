# Robot Self-Return Filtering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Filter robot-body lidar returns consistently and reduce MPPI safety margin to 0.04 m.

**Architecture:** Add a small footprint predicate in the MPPI costmap library and equivalent use in the global dynamic obstacle adapter. Apply it after lidar-to-base transformation and before inserting obstacles.

**Tech Stack:** C++17, CMake/CTest, Dora YAML.

## Global Constraints

- No ROS runtime dependency.
- `SAFETY_MARGIN` must be `0.04`.
- External obstacles outside the footprint must remain visible.
- Do not create Git commits.

---

### Task 1: MPPI footprint filter

**Files:**
- Modify: `planning/mppi_local_planner_dora_node/include/nav2_mppi_port/costmap.hpp`
- Modify: `planning/mppi_local_planner_dora_node/src/costmap.cpp`
- Modify: `planning/mppi_local_planner_dora_node/tests/costmap_test.cpp`
- Modify: `planning/mppi_local_planner_dora_node/src/mppi_local_planner_dora_node.cpp`

**Interfaces:**
- Produces: `bool insideRobotFootprint(const ObstaclePoint &, const FootprintConfig &)`

- [ ] Add failing tests for inside and outside footprint points.
- [ ] Run the costmap test and verify the new assertion fails because the API is absent.
- [ ] Implement the predicate and filter transformed scan points.
- [ ] Run all MPPI tests and verify they pass.

### Task 2: Global dynamic obstacle consistency

**Files:**
- Modify: `planning/adora_nano_global_planner_node/src/dora_planning_node.cpp`
- Modify: the existing global planner test covering scan obstacle insertion.

**Interfaces:**
- Consumes the same footprint dimensions and padding environment variables as MPPI.

- [ ] Add a failing regression showing a base-footprint return must not occupy the robot start cell.
- [ ] Implement filtering after lidar-to-base transformation.
- [ ] Run global planner tests and verify they pass.

### Task 3: Production configuration and verification

**Files:**
- Modify: `apps/adora_nano_navigation/adora_nano_navigation.yml`
- Modify: `apps/adora_nano_navigation/adora_nano_localization_navigation.yml`
- Modify: `apps/adora_nano_navigation/run_navigation.py`

- [ ] Set `SAFETY_MARGIN` to `0.04` in generated and static navigation flows.
- [ ] Add identical footprint dimensions and padding to both planners.
- [ ] Build all changed targets.
- [ ] Run MPPI, global planner, and supervisor test suites.
- [ ] Validate a generated Dora navigation flow without starting hardware.
