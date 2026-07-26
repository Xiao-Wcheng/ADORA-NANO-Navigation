# Dora Navigation Project Closeout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the standalone navigation tree into a clean, RPP-only, ROS-free deliverable whose build, tests, documentation, and terminal stop behavior agree.

**Architecture:** Keep the existing Karto localization/mapping, Boost.Graph A*, Nav2 RPP port, velocity smoother, and supervisor safety gate. Remove disconnected DWB/MPPI implementations and make structural tests enforce the production boundary. Keep terminal zero-command enforcement in the supervisor and add regression coverage at the supervisor policy boundary.

**Tech Stack:** C++17/CMake/CTest, Python 3, YAML, Dora C API, Bash.

## Global Constraints

- Work only in `/home/ubuntu2204/adora_nano_dora_navigation`.
- Keep RPP as the only production local controller.
- Do not add ROS runtime dependencies.
- Do not change vehicle geometry, safety margins, inflation, or motion tuning.
- Do not start hardware dataflows or command the chassis.
- Do not create Git commits.

---

### Task 1: Establish RPP-only structural contracts

**Files:**
- Modify: `tests/test_entrypoints.py`
- Modify: `tests/test_standalone_paths.py`
- Move: `apps/adora_nano_navigation/tests/test_mppi_production_wiring.py` to `apps/adora_nano_navigation/tests/test_rpp_production_wiring.py`

**Interfaces:**
- Consumes: root build script and four production YAML files.
- Produces: executable structural tests that require RPP and reject DWB/MPPI production components.

- [ ] Update tests to require `planning/rpp_local_controller_dora_node`, the velocity smoother, and RPP configuration.
- [ ] Add assertions that retired DWB/MPPI directories and production references are absent.
- [ ] Run the tests and confirm RED because old directories and documentation still exist.

### Task 2: Remove retired implementations and stale production references

**Files:**
- Delete: `planning/dwb_local_planner_dora_node/`
- Delete: `planning/mppi_local_planner_dora_node/`
- Modify: `docs/standalone_navigation_manifest.txt`
- Modify: any build/check scripts still referring to retired planners.

**Interfaces:**
- Consumes: failing RPP-only structural contracts.
- Produces: a production tree containing one local controller implementation.

- [ ] Resolve exact retired paths with `find` and production references with recursive text search.
- [ ] Delete only the two confirmed retired planner directories.
- [ ] Regenerate or edit the standalone manifest to contain the retained tree only.
- [ ] Run structural tests and confirm directory/reference assertions pass.

### Task 3: Enforce terminal stop behavior

**Files:**
- Modify: `planning/nav_supervisor_dora_node/src/supervisor_goal.hpp`
- Modify: `planning/nav_supervisor_dora_node/src/supervisor_goal.cpp`
- Modify: `planning/nav_supervisor_dora_node/src/nav_supervisor_dora_node.cpp`
- Modify: `planning/nav_supervisor_dora_node/tests/nav_supervisor_goal_test.cpp`

**Interfaces:**
- Consumes: supervisor goal result and terminal latch state.
- Produces: a pure policy decision that requires zero safe output whenever `REACHED` is latched.

- [ ] Add a failing unit case proving a latched `REACHED` state rejects a nonzero local command.
- [ ] Run the supervisor test and confirm RED for the missing policy.
- [ ] Implement the smallest pure helper and use it in the safe-command output path.
- [ ] Run supervisor CTest and confirm GREEN.

### Task 4: Align documentation and open-source provenance

**Files:**
- Modify: `README.md`
- Modify: `docs/COMPONENTS.md`
- Modify: `docs/OPERATIONS.md`
- Modify: `docs/OPEN_SOURCE.md`
- Modify: `docs/VALIDATION.md`
- Retain: `planning/rpp_local_controller_dora_node/UPSTREAM.md`
- Retain: `planning/rpp_local_controller_dora_node/LICENSE.nav2-rpp-apache-2.0`

**Interfaces:**
- Consumes: final production architecture and test names.
- Produces: current operator and provenance documentation with no DWB/MPPI production claims.

- [ ] Replace current-architecture DWB/MPPI descriptions with RPP.
- [ ] Mark historical design documents as non-production records or exclude them from production scans.
- [ ] Update validation results only after fresh verification.
- [ ] Run production reference scans and confirm no stale current-architecture claims.

### Task 5: Full non-motor verification

**Files:**
- Verify: `scripts/build_all.sh`
- Verify: `scripts/check_ready.sh`
- Verify: production YAML files under `apps/adora_nano_navigation/`

**Interfaces:**
- Consumes: cleaned source tree.
- Produces: fresh build and verification evidence.

- [ ] Run `./scripts/stop_all.sh` and verify no control processes remain.
- [ ] Run `./scripts/build_all.sh`.
- [ ] Run all CTest suites under retained production nodes with `--output-on-failure`.
- [ ] Run Python structural and application wiring tests.
- [ ] Run `./scripts/check_ready.sh`.
- [ ] Validate all production Dora YAML files without starting them.
- [ ] Scan production executables with `ldd` and reject ROS-linked libraries.
- [ ] Update `docs/VALIDATION.md` with the fresh results and rerun documentation/structure tests.
- [ ] Verify again that no navigation or chassis control processes are running.
